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
#include "activity.hpp"

#undef ACTIVITY_LOG_NAME
#define ACTIVITY_LOG_NAME "role-selection"

#undef  XPCC_LOG_LEVEL
#define XPCC_LOG_LEVEL xpcc::log::WARNING

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::RoleSelectionActivity
xpcc::R2MAC<Nrf24Data, Parameters>::roleSelectionActivity;

template<typename Nrf24Data, class Parameters>
xpcc::ResumableResult<void>
xpcc::R2MAC<Nrf24Data, Parameters>::RoleSelectionActivity::update()
{
	static xpcc::PeriodicTimer rateLimiter(500);
	if(rateLimiter.execute()) {
		R2MAC_LOG_INFO << COLOR_BLUE << "Activity: " << toStr(activity)
		               << COLOR_END << xpcc::endl;
	}

	ACTIVITY_GROUP_BEGIN(0)
	{
		DECLARE_ACTIVITY(Activity::Init)
		{
			clearAssociation();
			memberCount = 0;
			beaconQueue.clear();
			CALL_ACTIVITY(Activity::ListenForBeacon);
		}

		DECLARE_ACTIVITY(Activity::ListenForBeacon)
		{
			// listen listen for a beacon frame
			timeoutUs.restart(1.5 * timeMaxSuperFrameUs);

			while(not timeoutUs.execute()) {
				if(not beaconQueue.isEmpty()) {
					CALL_ACTIVITY(Activity::BecomeMember);
				}

				RF_YIELD();
			}

			// no beacon received in time ...
			CALL_ACTIVITY(Activity::TryBecomingCoordinator);
		}

		DECLARE_ACTIVITY(Activity::TryBecomingCoordinator)
		{
			// flip a coin if becoming coordinator:
			//   yes: listen for random period, if no beacon received, become
			//        coordinator
			//   no:  listen for beacon again

			if(randomRange(0, 100) < Parameters::coordinatorElectionProbabilityPercent) {
				// not becoming a coordinator this time
				CALL_ACTIVITY(Activity::ListenForBeacon);
			}


			{
				const uint32_t delaySlots = randomRange(
				            Parameters::minSlotsAfterCoordinatorElection,
				            Parameters::maxSlotsAfterCoordinatorElection);

				const uint32_t delay = timeAssociationSlotUs * delaySlots;

				R2MAC_LOG_DEBUG << "Waiting for " << delaySlots << " slots "
				                << "before becoming coordinator" << xpcc::endl;

				timeoutUs.restart(delay);
			}

			while(not timeoutUs.execute()) {
				if(not beaconQueue.isEmpty()) {
					// another node has become a coordinator and emitted a beacon
					CALL_ACTIVITY(Activity::BecomeMember);
				}

				RF_YIELD();
			}

			CALL_ACTIVITY(Activity::BecomeCoordinator);
		}

		DECLARE_ACTIVITY(Activity::BecomeCoordinator)
		{
			R2MAC_LOG_INFO << "Becoming Coordinator!" << xpcc::endl;

			role = Role::Coordinator;
			ACTIVITY_GROUP_EXIT(Activity::Init, true);
		}

		DECLARE_ACTIVITY(Activity::BecomeMember)
		{
			R2MAC_LOG_INFO << "Becoming Member!" << xpcc::endl;

			role = Role::Member;
			ACTIVITY_GROUP_EXIT(Activity::Init, true);
		}
	}

	ACTIVITY_GROUP_END();
}
