// coding: utf-8
/* Copyright (c) 2017, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC__NRF24_MAC_R2MAC_HPP
#   error "Don't include this file directly, use 'r2mac.hpp' instead!"
#endif

#include <xpcc/processing.hpp>
#include <xpcc/debug/logger.hpp>
#include "r2mac.hpp"

#undef ACTIVITY_LOG_NAME
#define ACTIVITY_LOG_NAME "coordinator"

#undef  XPCC_LOG_LEVEL
#define XPCC_LOG_LEVEL xpcc::log::DEBUG

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::CoordinatorActivity
xpcc::R2MAC<Nrf24Data, Parameters>::coordinatorActivity;

template<typename Nrf24Data, class Parameters>
xpcc::ResumableResult<void>
xpcc::R2MAC<Nrf24Data, Parameters>::CoordinatorActivity::update()
{
	static xpcc::PeriodicTimer rateLimiter(500);
	if(rateLimiter.execute()) {
		R2MAC_LOG_INFO << COLOR_YELLOW << "Activity: " << toStr(activity)
		               << COLOR_END << xpcc::endl;
	}

	ACTIVITY_GROUP_BEGIN(0)
	{
		DECLARE_ACTIVITY(Activity::Init)
		{
			clearAssociation();

			// we're the coordinator now
			coordinatorAddress = getAddress();

			// clear stale control packets in queues
			beaconQueue.clear();
			associationQueue.clear();

			// Reset member list and lease timers
			memberCount = 0;
			for(uint8_t i = 0; i < Parameters::maxMembers; i++) {
				memberLeaseTimeouts[i].stop();
				memberList[i] = 0;
			}

			CALL_ACTIVITY(Activity::SendBeacon);
		}

		DECLARE_ACTIVITY(Activity::SendBeacon)
		{
			{	// prepare beacon frame
				auto beaconPacket = reinterpret_cast<Packet*>(&packetNrf24Data);
				auto beaconFrame = reinterpret_cast<typename Frames::Beacon*>(beaconPacket->payload);

				beaconPacket->setDestination(Nrf24Data::getBroadcastAddress());
				beaconPacket->setType(Packet::Type::Beacon);

				beaconFrame->memberCount = memberCount;
				for (uint8_t i = 0; i < memberCount; i++) {
					beaconFrame->members[i] = memberList[i];
				}
			}

			// busy-wait until we can send a packet, we don't want to yield
			// because we want to send the beacon as soon as possible
			waitUntilReadyToSend();

			if(Nrf24Data::sendPacket(packetNrf24Data)) {
				// beacon has now been sent
				timeLastBeacon = MicroSecondsClock::now();
			} else {
				R2MAC_LOG_ERROR << "Unable to send beacon" << xpcc::endl;

				// try again
				CALL_ACTIVITY(Activity::SendBeacon);
			}

			{	// Debug output
				const uint32_t superFrameDurationMs =
				        (getSuperFrameDurationUs(memberCount) / 1000);
				const uint32_t superFrameDurationDiffUs =
				        getSuperFrameDurationUs(memberCount) - (superFrameDurationMs * 1000);

				R2MAC_LOG_INFO << "Super Frame duration: ";
				XPCC_LOG_INFO.printf("%u.%03u ms\n", superFrameDurationMs, superFrameDurationDiffUs);

				R2MAC_LOG_INFO << "Member count: " << memberCount << xpcc::endl;
				R2MAC_LOG_INFO << "members: ";
				for(int i = 0; i < memberCount; i++) {
					XPCC_LOG_INFO.printf("0x%02x ", memberList[i]);
				}
				XPCC_LOG_INFO << xpcc::endl;
			}

			CALL_ACTIVITY(Activity::ListenForRequests);
		}

		DECLARE_ACTIVITY(Activity::ListenForRequests)
		{
			targetTimestamp = timeLastBeacon + timeAssociationPeriodUs;

			// Wait for association requests. Note: All Nrf24 packets are parsed
			//  (and pushed on a proper queue) inside the update() function.
			while(MicroSecondsClock::now() < targetTimestamp) {
				if(not beaconQueue.isEmpty()) {
					CALL_ACTIVITY(Activity::LeaveCoordinatorRole);
				}
				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::SendData);
		}

		DECLARE_ACTIVITY(Activity::SendData)
		{
			// Time when a packet ca be transmitted the latest to not violate
			// the guard interval.
			// Note: assumes frame transmission only lasts frameAirTimeUs, not
			// considering switch time nor retransmissions yet
			targetTimestamp = getStartOfOwnSlot() + timeDataTransmissionUs - frameAirTimeUs;

			while(inMySlot()) {
				if(not beaconQueue.isEmpty()) {
					CALL_ACTIVITY(Activity::LeaveCoordinatorRole);
				}

				if(not dataTXQueue.isEmpty()) {

					if(MicroSecondsClock::now() < targetTimestamp) {

						if(Nrf24Data::isReadyToSend() and Nrf24Data::sendPacket(dataTXQueue.getFront())) {
							dataTXQueue.removeFront();

							while(Nrf24Data::getFeedback().sendingFeedback == Nrf24Data::SendingFeedback::Busy) {
								Nrf24Data::update();
								RF_YIELD();
							}
						}
					}
				}

				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::ReceiveData);
		}

		DECLARE_ACTIVITY(Activity::ReceiveData)
		{
			// End of Super Frame
			targetTimestamp = timeLastBeacon + getSuperFrameDurationUs(memberCount);

			while(MicroSecondsClock::now() < targetTimestamp) {

				if(not beaconQueue.isEmpty()) {
					CALL_ACTIVITY(Activity::LeaveCoordinatorRole);
				}

				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::UpdateNodeList);
		}

		DECLARE_ACTIVITY(Activity::UpdateNodeList)
		{
			while(not associationQueue.isEmpty()) {
				NodeAddress newMemberAddress = associationQueue.getFront();
				associationQueue.removeFront();

				const uint8_t slotNumber = getDataSlot(newMemberAddress);
				const bool alreadyAssociated = (slotNumber == 0);

				if(alreadyAssociated) {
					// Create new node entry and its timeout
					if (memberCount < Parameters::maxMembers) {
						memberList[memberCount] = newMemberAddress;
						memberLeaseTimeouts[memberCount].restart(Parameters::timeNodeLeaseUs);
						memberCount++;

						R2MAC_LOG_INFO << "Register new member node: 0x"
						               << xpcc::hex << newMemberAddress
						               << xpcc::ascii << xpcc::endl;
					} else {
						R2MAC_LOG_ERROR << "Unable to register new member: 0x"
						                << xpcc::hex << newMemberAddress
						                << xpcc::ascii << ". Reached maximum "
						                << "amount of members." << xpcc::endl;
					}
				} else {
					R2MAC_LOG_INFO << "Reset lease timer for the existing member: 0x"
					               << xpcc::hex << newMemberAddress << xpcc::ascii
					               << xpcc::endl;

					const uint8_t index = slotNumber - 1;
					memberLeaseTimeouts[index].restart(Parameters::timeNodeLeaseUs);
				}
			}

			// Check timers whether executed (node lease time)
			// Scan backwards, because when you relocate expired timer with the
			// last timer in a list you have to know whether relocated timer has
			// expired (no double check after reassigning).
			for (int8_t expiredNodeIndex = memberCount - 1; expiredNodeIndex >= 0; expiredNodeIndex--) {
				if (memberLeaseTimeouts[expiredNodeIndex].isExpired()) {
					const uint8_t lastNodeIndex = memberCount - 1;

					// Reduce amount of nodes
					memberCount--;

					if (expiredNodeIndex < lastNodeIndex) {
						// relocate last lease timeout
						memberLeaseTimeouts[expiredNodeIndex] = memberLeaseTimeouts[lastNodeIndex];

						// Switch node address inside node list between last and
						// expired node
						memberList[expiredNodeIndex] = memberList[lastNodeIndex];
					}
				}
			}

			CALL_ACTIVITY(Activity::SendBeacon);
		}

		DECLARE_ACTIVITY(Activity::LeaveCoordinatorRole)
		{
			// Beacon frame received while beeing coordinator - go to the member role
			role = Role::Member;
			ACTIVITY_GROUP_EXIT(Activity::Init, true);
		}
	}

	ACTIVITY_GROUP_END();
}
