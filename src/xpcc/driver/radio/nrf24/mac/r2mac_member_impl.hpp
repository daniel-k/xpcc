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

#undef ACTIVITY_LOG_NAME
#define ACTIVITY_LOG_NAME "member"

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
bool
xpcc::R2MAC<Nrf24Data, Parameters>::MemberActivity::inMySlot()
{
	const xpcc::Timestamp endOwnSlotTimestamp = getStartOfOwnSlot() +
	        timeDataSlotUs;

	const xpcc::Timestamp now = MicroSecondsClock::now();

	return (getStartOfOwnSlot() < now) and (now < endOwnSlotTimestamp);
}

template<typename Nrf24Data, class Parameters>
xpcc::ResumableResult<void>
xpcc::R2MAC<Nrf24Data, Parameters>::MemberActivity::update()
{
	static xpcc::PeriodicTimer rateLimiter(500);
	if(rateLimiter.execute()) {
		R2MAC_LOG_INFO << GREEN << "Activity: " << toStr(activity) << END << xpcc::endl;
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
			timeoutUs.restart(getSuperFrameDurationUs(memberCount) *
			                                  Parameters::maxMissedBeacons);

			// Wait for Beacon frame
			while(not timeoutUs.execute()) {
				if(not beaconQueue.isEmpty()) {
					{
						Packet& beaconPacket = beaconQueue.getFront();
						auto beaconFrame = reinterpret_cast<typename Frames::Beacon*>
						                            (beaconPacket.payload);

						XPCC_LOG_INFO.printf("Found coordinator 0x%02x\n", beaconPacket.getSource());
						updateNetworkInfo(beaconPacket.getSource(), *beaconFrame);
						beaconQueue.removeFront();

						// beaconPacket and beaconFrame are not valid anymore
					}

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
			// Select a random association slot
			associationSlot = randomRange(0, Parameters::associationSlots - 1);

			R2MAC_LOG_INFO << "Waiting for " << associationSlot << " slots "
			               << "to send association request" << xpcc::endl;

			targetTimestamp = timeLastBeacon +
			        (timeAssociationSlotUs * associationSlot);

			while(MicroSecondsClock::now() < targetTimestamp) {
				if(not beaconQueue.isEmpty()) {
					CALL_ACTIVITY(Activity::ReceiveBeacon);
				}

				RF_YIELD();
			}

			{	// assemble and send association request
				Nrf24DataPacket packetNrf24Data;
				auto associationPacket = reinterpret_cast<Packet*>(&packetNrf24Data);

				associationPacket->setDestination(coordinatorAddress);
				associationPacket->setType(Packet::Type::AssociationRequest);

				Nrf24Data::sendPacket(packetNrf24Data);
			}

			RF_WAIT_UNTIL(Nrf24Data::getFeedback().sendingFeedback != Nrf24Data::SendingFeedback::Busy);

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
			// End of Super Frame
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
			// Time when a packet ca be transmitted the latest to not violate
			// the guard interval.
			// Note: assumes frame transmission only lasts frameAirTimeUs, not
			// considering switch time nor retransmissions yet
			targetTimestamp = getStartOfOwnSlot() + timeTransmissionUs - frameAirTimeUs;

			while(inMySlot()) {

				if (not dataTXQueue.isEmpty()) {
					if(MicroSecondsClock::now() < targetTimestamp) {

						Nrf24Data::sendPacket(dataTXQueue.getFront());
						dataTXQueue.removeFront();

						RF_WAIT_UNTIL(Nrf24Data::getFeedback().sendingFeedback != Nrf24Data::SendingFeedback::Busy);
					}
				}
				RF_YIELD();
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
