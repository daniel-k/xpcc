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
#define ACTIVITY_LOG_NAME "member"

#undef  XPCC_LOG_LEVEL
#define XPCC_LOG_LEVEL xpcc::log::INFO

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::MemberActivity
xpcc::R2MAC<Nrf24Data, Parameters>::memberActivity;

template<typename Nrf24Data, class Parameters>
xpcc::ResumableResult<void>
xpcc::R2MAC<Nrf24Data, Parameters>::MemberActivity::update()
{
	static xpcc::PeriodicTimer rateLimiter(500);
	if(rateLimiter.execute()) {
		R2MAC_LOG_INFO << COLOR_GREEN << "Activity: " << toStr(activity)
		               << COLOR_END << xpcc::endl;
	}

	ACTIVITY_GROUP_BEGIN(0)
	{
		DECLARE_ACTIVITY(Activity::Init)
		{
			memberCount = 0;
			CALL_ACTIVITY(Activity::ReceiveBeacon);
		}

		DECLARE_ACTIVITY(Activity::ReceiveBeacon)
		{
			// we need to check our associate with every beacon
			clearAssociation();

			{
				// worst case assumption if no previous member count is known
				const uint32_t currentMemberCount =
				        (memberCount != 0) ? memberCount: Parameters::maxMembers;

				const uint32_t superFrameDurationUs =
				        getSuperFrameDurationUs(currentMemberCount);

				// maximum amount of time to wait for a beacon frame
				timeoutUs.restart(superFrameDurationUs * Parameters::maxMissedBeacons);
			}

			// wait for beacon frame
			while(not timeoutUs.isExpired()) {
				if(not beaconQueue.isEmpty()) {
					{
						// get from the back of the queue, we only care about
						// the most recently received beacon
						Packet& beaconPacket = beaconQueue.getBack();
						auto beaconFrame = reinterpret_cast<typename Frames::Beacon*>
						                            (beaconPacket.payload);

						// parse beacon frame
						coordinatorAddress = beaconPacket.getSource();
						memberCount = beaconFrame->memberCount;
						memcpy(&memberList, beaconFrame->members, memberCount);
						ownDataSlot = getDataSlot(getAddress());
					}

					// drop all other beacons
					beaconQueue.clear();

					CALL_ACTIVITY(Activity::CheckAssociation);
				}

				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::LeaveNetwork);
		}

		DECLARE_ACTIVITY(Activity::CheckAssociation)
		{
			// check if associated to the network
			if(not isAssociated()) {
				CALL_ACTIVITY(Activity::Associate);
			}

			if(leaseTimeoutUs.isExpired()) {
				CALL_ACTIVITY(Activity::Associate);
			}

			CALL_ACTIVITY(Activity::WaitForDataSlots);
		}

		DECLARE_ACTIVITY(Activity::Associate)
		{
			{	// prepare association request frame
				auto associationPacket = reinterpret_cast<Packet*>(&packetNrf24Data);

				associationPacket->setDestination(coordinatorAddress);
				associationPacket->setType(Packet::Type::AssociationRequest);
			}

			// Select a random association slot
			associationSlot = randomRange(0, Parameters::associationSlots - 1);

			targetTimestamp = timeLastBeacon +
			        (timeAssociationSlotUs * associationSlot);

			// busy-wait for our slot (probably required for tight timing)
			while( (MicroSecondsClock::now() < targetTimestamp) or
			       (not Nrf24Data::isReadyToSend())) {

				// update required for assessment of isReadyToSend()
				Nrf24Data::update();
			}

			// check if the association request still fits into our slot
			targetTimestamp = targetTimestamp +
			                    (timeAssociationTransmissionUs - frameAirTimeUs);
			if(MicroSecondsClock::now() < targetTimestamp ) {

				if(Nrf24Data::sendPacket(packetNrf24Data)) {
					// wait for the outcome of our association attempt, timing
					// not so crucial anymore
					RF_WAIT_UNTIL(Nrf24Data::isReadyToSend());

					switch(Nrf24Data::getFeedback().sendingFeedback) {
					case Nrf24Data::SendingFeedback::FinishedAck:
					case Nrf24Data::SendingFeedback::DontKnow:
						// reset lease timeout if it was successful or we cannot know
						leaseTimeoutUs.restart(Parameters::timeNodeLeaseUpdateUs);
						break;
					default:
						// not resetting lease because renewal failed
						break;
					}
				} else {
					R2MAC_LOG_ERROR << "Unable to send association request"
					                << xpcc::endl;
				}
			} else {
				R2MAC_LOG_ERROR << "We missed our association slot (it was #"
				                << associationSlot << ") :(" << xpcc::endl;
			}

			if(isAssociated()) {
				// we already have been associated and only sent an association
				// request for lease renewal
				CALL_ACTIVITY(Activity::WaitForDataSlots);
			} else {
				// wait for beacon
				CALL_ACTIVITY(Activity::ReceiveBeacon);
			}
		}

		DECLARE_ACTIVITY(Activity::WaitForDataSlots)
		{
			targetTimestamp = timeLastBeacon + timeAssociationPeriodUs;

			while(MicroSecondsClock::now() < targetTimestamp) {
				if(not beaconQueue.isEmpty()) {
					CALL_ACTIVITY(Activity::ReceiveBeacon);
				}

				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::ReceiveData);
		}

		DECLARE_ACTIVITY(Activity::ReceiveData)
		{
			// end of Super Frame
			targetTimestamp = timeLastBeacon + getSuperFrameDurationUs(memberCount);

			while(MicroSecondsClock::now() < targetTimestamp) {

				if(not beaconQueue.isEmpty()) {
					CALL_ACTIVITY(Activity::ReceiveBeacon);
				}

				if(inMySlot()) {
					CALL_ACTIVITY(Activity::OwnSlot);
				}

				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::ReceiveBeacon);
		}

		DECLARE_ACTIVITY(Activity::OwnSlot)
		{
			// Time when a packet can be transmitted at the latest to not
			// violate the guard interval.
			// Note: assumes frame transmission only lasts frameAirTimeUs, not
			// considering switch time nor retransmissions yet
			targetTimestamp = getStartOfOwnSlot() + timeDataTransmissionUs - frameAirTimeUs;

			while(inMySlot()) {
				if (not dataTXQueue.isEmpty()) {

					// wait until we can really transmit
					while(not Nrf24Data::isReadyToSend()) {
						Nrf24Data::update();
					}

					if(MicroSecondsClock::now() < targetTimestamp) {
						if(Nrf24Data::sendPacket(dataTXQueue.getFront())) {
							dataTXQueue.removeFront();
						}
					}
				}

				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::ReceiveData);
		}

		DECLARE_ACTIVITY(Activity::LeaveNetwork)
		{
			clearAssociation();

			role = Role::None;
			ACTIVITY_GROUP_EXIT(Activity::Init, true);
		}
	}

	ACTIVITY_GROUP_END();
}
