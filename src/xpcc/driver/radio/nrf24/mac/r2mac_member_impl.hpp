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
typename xpcc::R2MAC<Nrf24Data, Parameters>::MemberActivity
xpcc::R2MAC<Nrf24Data, Parameters>::memberActivity;


template<typename Nrf24Data, class Parameters>
void
xpcc::R2MAC<Nrf24Data, Parameters>::MemberActivity::initialize()
{
	activity = Activity::Init;
}

template<typename Nrf24Data, class Parameters>
xpcc::ResumableResult<void>
xpcc::R2MAC<Nrf24Data, Parameters>::MemberActivity::update()
{
	ACTIVITY_GROUP_BEGIN(0)
	{
		DECLARE_ACTIVITY(Activity::Init)
		{
			CALL_ACTIVITY(Activity::ReceiveBeacon);
		}

		DECLARE_ACTIVITY(Activity::ReceiveBeacon)
		{
			if (memberCount == 0) {
				timeoutUs.restart(timeMaxSuperFrameUs);
			} else {
				timeoutUs.restart(getSuperFrameDurationUs(memberCount) *
												  Parameters::maxMissedBeacons);
			}

			// Wait for Beacon frame
			while(not timeoutUs.execute()) {
				if(not beaconQueue.empty()) {
					CALL_ACTIVITY(Activity::CheckAssociation);
				}

				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::LeaveNetwork);
		}

		DECLARE_ACTIVITY(Activity::CheckAssociation)
		{
			// check if associated to the network
			// TODO: New packet structure

			{
				const auto beaconPacket = reinterpret_cast<Packet*>
				                            (&beaconQueue.getFront());
				const auto beaconFrame = reinterpret_cast<typename Frames::Beacon*>
				                            (beaconPacket->payload);

				updateNetworkInfo(beaconPacket->getSource(), *beaconFrame);
				beaconQueue.removeFront();

				// beaconPacket and beaconFrame are not valid anymore
			}

			if(ownDataSlot == 0) {
				CALL_ACTIVITY(Activity::Associate);
			}

			if(leaseTimeoutUs.isExpired()) {
				CALL_ACTIVITY(Activity::Associate);
			}

			associationSlot = 0;	// why?
			CALL_ACTIVITY(Activity::WaitForDataSlots);
		}

		DECLARE_ACTIVITY(Activity::Associate)
		{
			// TODO: Maybe we would like to restart lease timeout on data frame transmission
			leaseTimeoutUs.restart(Parameters::timeNodeLeaseUpdateUs);

			// Select a random association slot
			associationSlot = randomRange(1, Parameters::associationSlots);

			R2MAC_LOG_INFO << "Waiting for " << associationSlot << " slots "
			               << "to send association request" << xpcc::endl;

			{
				const uint32_t delay = timeAssociationSlotUs * (associationSlot - 1);
				timeoutUs.restart(delay);
			}

			while(not timeoutUs.execute()) {
				if(not beaconQueue.empty()) {
					CALL_ACTIVITY(Activity::ReceiveBeacon);
				}

				RF_YIELD();
			}

			Nrf24DataPacket packetNrf24Data;
			auto associationPacket = reinterpret_cast<Packet*>(&packetNrf24Data);
			auto associationFrame = reinterpret_cast<typename Frames::AssociationRequest*>
			                            (associationPacket->payload);

			associationPacket->setDestination(coordinatorAddress);
			associationPacket->setType(Packet::Type::AssociationRequest);

			Nrf24Data::sendPacket(packetNrf24Data);

			if (not isAssociated()) {
				// wait for beacon
				CALL_ACTIVITY(Activity::ReceiveBeacon);
			} else {
				// we already have been associated and only sent an association
				// request for lease renewal
				CALL_ACTIVITY(Activity::WaitForDataSlots);
			}
		}

		DECLARE_ACTIVITY(Activity::WaitForDataSlots)
		{
			{
				// Calculate StartOfDataSlots timeout
				const uint32_t delay = timeAssociationSlotUs *
				               (Parameters::associationSlots - associationSlot);

				// Wait for the beginning of data slots
				timeoutUs.restart(delay);
			}

			while(not timeoutUs.execute()) {
				if(not beaconQueue.empty()) {
					CALL_ACTIVITY(Activity::ReceiveBeacon);
				}
				RF_YIELD();
			}

			// Calculate EndOfDataSots timeout
			timeoutUs.restart(timeDataSlotUs * (memberCount + 1));

			// Calculate start of ownSlot
			ownSlotTimeoutUs.restart(timeDataSlotUs * ownDataSlot);

			CALL_ACTIVITY(Activity::ReceiveData);
		}

		DECLARE_ACTIVITY(Activity::ReceiveData)
		{
			// Wait for ownSlot
			while(not timeoutUs.execute()) {
				if(not beaconQueue.empty()) {
					CALL_ACTIVITY(Activity::ReceiveBeacon);
				}

				if(ownSlotTimeoutUs.execute()) {
					CALL_ACTIVITY(Activity::OwnSlot);
				}

				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::ReceiveBeacon);
		}

		DECLARE_ACTIVITY(Activity::OwnSlot)
		{
			ownSlotTimeoutUs.restart(timeDataSlotUs);

			while(not ownSlotTimeoutUs.execute()) {
				// Check if TXqueue empty
				if (not dataTXQueue.isEmpty()) {
					// Send packet
					// TODO: Retransmissions not included in time calculation.
					if (ownSlotTimeoutUs.remaining() > frameAirTimeUs) {
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

			CALL_ACTIVITY(Activity::ReceiveData);
		}

		DECLARE_ACTIVITY(Activity::LeaveNetwork)
		{
			ownDataSlot = 0;
			// No beacons within timeout - go to the role selection
			role = Role::None;
			ACTIVITY_GROUP_EXIT(Activity::Init, true);
		}
	}

	ACTIVITY_GROUP_END();
}
