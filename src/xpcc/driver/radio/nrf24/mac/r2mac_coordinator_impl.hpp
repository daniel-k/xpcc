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
#include "r2mac.hpp"

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::CoordinatorActivity
xpcc::R2MAC<Nrf24Data, Parameters>::coordinatorActivity;


template<typename Nrf24Data, class Parameters>
void
xpcc::R2MAC<Nrf24Data, Parameters>::CoordinatorActivity::initialize()
{
	activity = Activity::Init;
}

template<typename Nrf24Data, class Parameters>
xpcc::ResumableResult<void>
xpcc::R2MAC<Nrf24Data, Parameters>::CoordinatorActivity::update()
{
	ACTIVITY_GROUP_BEGIN(0)
	{
		DECLARE_ACTIVITY(Activity::Init)
		{
			uint8_t i;
			memberCount = 0;

			for(i = 0; i < Parameters::maxMembers; i++){
				// Stop timeout timers
				memberLeaseTimeouts[i].stop();

				// Reset node list
				memberList[i] = 0;
			}

			// Clear association requests from the past
			associationQueue.clear();

			CALL_ACTIVITY(Activity::SendBeacon);
		}

		DECLARE_ACTIVITY(Activity::SendBeacon)
		{
			{
				typename Nrf24Data::Packet newBeaconPacket;
				auto newBeaconFrame = reinterpret_cast<typename Frame::Beacon*>(newBeaconPacket.payload.data);

				// Construct Nrf24 packet
				newBeaconPacket.dest = Nrf24Data::getBroadcastAddress();
				newBeaconPacket.payload.length = sizeof(Frame::Beacon);

				// Create beacon frame
				newBeaconFrame->memberCount = memberCount;
				for (uint8_t i = 0; i < memberCount; i++) {
					newBeaconFrame->members[i] = memberList[i];
				}

				// Enqueue new packet
				Nrf24Data::sendPacket(newBeaconPacket);
			}

			CALL_ACTIVITY(Activity::ListenForRequests);
		}

		DECLARE_ACTIVITY(Activity::ListenForRequests)
		{
			timeoutUs.restart(timeAssociationSlotUs * Parameters::associationSlots);

			// Wait for association requests. Note: All Nrf24 packets are parsed
			//  (and pushed on a proper queue) inside .update() function.
			while(not timeoutUs.execute()) {
				if(not beaconQueue.empty()) {
					CALL_ACTIVITY(Activity::LeaveCoordinatorRole);
				}

				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::SendData);
		}

		DECLARE_ACTIVITY(Activity::SendData)
		{
			timeoutUs.restart(timeDataSlotUs - timeGuardUs);

			while(not timeoutUs.execute()) {
				if(not beaconQueue.empty()) {
					CALL_ACTIVITY(Activity::LeaveCoordinatorRole);
				}

				if(not dataTXQueue.isEmpty()) {
					// TODO: Retransmissions not included in time calculation.
					if (timeoutUs.remaining() > frameAirTimeUs) {
						// Enqueue new packet
						Nrf24Data::sendPacket(dataTXQueue.getFront());

						// Remove packet from TX queue
						dataTXQueue.removeFront();
					} else {
						// not enough time to send packet anymore
						RF_YIELD();
					}
				} else {
					RF_YIELD();
				}
			}

			// Skip to the members data slots
			CALL_ACTIVITY(Activity::ReceiveData);
		}

		DECLARE_ACTIVITY(Activity::ReceiveData)
		{
			// Wait for data transmissions.
			timeoutUs.restart(timeDataSlotUs * memberCount);

			while(not timeoutUs.execute()) {
				if(not beaconQueue.empty()) {
					CALL_ACTIVITY(Activity::LeaveCoordinatorRole);
				}

				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::UpdateNodeList);
		}

		DECLARE_ACTIVITY(Activity::UpdateNodeList)
		{
			NodeAddress updateNode;
			uint8_t newNode;

			while(not associationQueue.isEmpty()) {
				updateNode = associationQueue.getFront();
				associationQueue.removeFront();

				// Check if already associated
				newNode = getDataSlot(updateNode);

				// Create new node entry and its timeout
				if (newNode) {
					if (memberCount < Parameters::maxMembers) {
						memberList[memberCount] = newNode;
						memberLeaseTimeouts[memberCount].restart(Parameters::timeNodeLeaseUs);
						memberCount++;

						R2MAC_LOG_INFO << "Register new member node: 0x" << xpcc::hex
									   << updateNode << xpcc::ascii << xpcc::endl;
					} else {
						R2MAC_LOG_ERROR << "Unable to associate new member: 0x" << xpcc::hex
									   << updateNode << xpcc::ascii
									   << ". Reached maximum amount of members." << xpcc::endl;
					}
				} else {
					// Update lease timeout
					memberLeaseTimeouts[newNode].restart(Parameters::timeNodeLeaseUs);
					R2MAC_LOG_INFO << "Reset lease timer for the existing node: 0x" << xpcc::hex
								   << updateNode << xpcc::ascii << xpcc::endl;

				}
			}

			// Check timers whether executed (node lease time)
			// Scan backwards, because when you relocate expired timer with the last timer in a list
			//  you have to know whether relocated timer has expired (no double check after reassigning).
			for (int8_t expiredNodeIndex = memberCount - 1; expiredNodeIndex >= 0; expiredNodeIndex--) {
				if (memberLeaseTimeouts[expiredNodeIndex].isExpired()) {
					const uint8_t lastNodeIndex = memberCount - 1;

					// Reduce amount of nodes
					memberCount--;

					if (expiredNodeIndex < lastNodeIndex) {
						// relocate last lease timeout
						memberLeaseTimeouts[expiredNodeIndex] = memberLeaseTimeouts[lastNodeIndex];

						// Switch node address inside node list between last and expired node
						memberList[expiredNodeIndex] = memberList[lastNodeIndex];
					}
				}
			}

			CALL_ACTIVITY(Activity::SendBeacon);
		}

		DECLARE_ACTIVITY(Activity::LeaveCoordinatorRole)
		{
			ACTIVITY_GROUP_EXIT(Activity::Init, true);
		}
	}

	ACTIVITY_GROUP_END();
}
