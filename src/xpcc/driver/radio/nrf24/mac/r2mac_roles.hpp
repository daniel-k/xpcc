#ifndef XPCC__NRF24_MAC_R2MAC_ROLES_HPP
#define XPCC__NRF24_MAC_R2MAC_ROLES_HPP

#include <stdint.h>
#include <xpcc/architecture.hpp>

#include "r2mac_types.hpp"

namespace xpcc {

template<typename Config>
class R2MACRoles {
protected:

	enum class
	Role
	{
		None,
		Coordinator,
		Member
	};

	static const char*
	toStr(Role role) {
		switch(role) {
		case Role::Coordinator:	return "Coordinator";
		case Role::Member:		return "Member";
		case Role::None:		return "None";
		default:				return "Invalid"; }
	}


    class Activities {
    protected:

        class MemberActivity : private xpcc::Resumable<1>
        {
        public:
            MemberActivity()
            { activity = Activity::Init; }

            xpcc::ResumableResult<void>
            update();

        private:
            enum class
            Activity
            {
                Init,
                ReceiveBeacon,
                CheckAssociation,
                Associate,
                WaitForDataSlots,
                ReceiveData,
                OwnSlot,
                LeaveNetwork,
            };

            static const char*
            toStr(Activity activity) {
                switch(activity) {
                case Activity::Init:				return "Init";
                case Activity::ReceiveBeacon:		return "ReceiveBeacon";
                case Activity::CheckAssociation:	return "CheckAssociation";
                case Activity::Associate:			return "Associate";
                case Activity::WaitForDataSlots:	return "WaitForDataSlots";
                case Activity::ReceiveData:			return "ReceiveData";
                case Activity::OwnSlot:				return "OwnSlot";
                case Activity::LeaveNetwork:		return "LeaveNetwork";
                default:							return "Invalid"; }
            }

            Activity activity;
            typename Config::TimeoutUs timeoutUs;
            typename Config::TimeoutUs leaseTimeoutUs;
            int32_t associationSlot;
            xpcc::Timestamp targetTimestamp;

        };


        class CoordinatorActivity : private xpcc::Resumable<1>
        {
        public:
            CoordinatorActivity()
            { activity = Activity::Init; }

            xpcc::ResumableResult<void>
            update();

        private:
            enum class
            Activity
            {
                Init,
                SendBeacon,
                ListenForRequests,
                SendData,
                ReceiveData,
                UpdateNodeList,
                LeaveCoordinatorRole,
            };

            static const char*
            toStr(Activity activity) {
                switch(activity) {
                case Activity::Init:					return "Init";
                case Activity::SendBeacon:				return "SendBeacon";
                case Activity::ListenForRequests:		return "ListenForRequests";
                case Activity::SendData:				return "SendData";
                case Activity::ReceiveData:				return "ReceiveData";
                case Activity::UpdateNodeList:			return "UpdateNodeList";
                case Activity::LeaveCoordinatorRole:	return "LeaveCoordinatorRole";
                default:								return "Invalid";  }
            }

            Activity activity;
            typename Config::TimeoutUs timeoutUs;
            typename Config::NodeTimerList memberLeaseTimeouts;
            xpcc::Timestamp targetTimestamp;
        };


        class RoleSelectionActivity : private xpcc::Resumable<1>
        {
        public:
            RoleSelectionActivity()
            { activity = Activity::Init; }

            xpcc::ResumableResult<void>
            update();

        private:
            enum class
            Activity
            {
                Init,
                ListenForBeacon,
                BecomeMember,
                CheckBecomingCoordinator,
                TryBecomingCoordinator,
                BecomeCoordinator,
            };


            static const char*
            toStr(Activity activity) {
                switch(activity) {
                case Activity::Init:						return "Init";
                case Activity::ListenForBeacon:				return "ListenForBeacon";
                case Activity::BecomeMember:				return "BecomeMember";
                case Activity::CheckBecomingCoordinator:	return "CheckBecomingCoordinator";
                case Activity::TryBecomingCoordinator:		return "TryBecomingCoordinator";
                case Activity::BecomeCoordinator:			return "BecomeCoordinator";
                default:									return "Invalid"; }
            }

            Activity activity;
            typename Config::TimeoutUs timeoutUs;
        };

    };

};

} // namespace xpcc

#endif // XPCC__NRF24_MAC_R2MAC_ROLES_HPP
