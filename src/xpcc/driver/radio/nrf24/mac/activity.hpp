
#ifndef ROBOTS_ACTIVITIES_MACROS_HPP
#define ROBOTS_ACTIVITIES_MACROS_HPP

#include <xpcc/debug/logger.hpp>

/// Redefine this with the name of the activity
#define ACTIVITY_LOG_NAME ""

/// Gets called upon enterting the activity group
#define ACTIVITY_LOG_BEGIN(activity) \
	    XPCC_LOG_DEBUG << XPCC_FILE_INFO << "[sm:" ACTIVITY_LOG_NAME "] " << "Enter with " << toStr(activity) << xpcc::endl

/// Gets called upon exiting the activity group
#define ACTIVITY_LOG_EXIT(activity) \
	    XPCC_LOG_DEBUG << XPCC_FILE_INFO << "[sm:" ACTIVITY_LOG_NAME "] " << "Exit with " << toStr(activity) << xpcc::endl

/// Gets called when a new activity has been called
#define ACTIVITY_LOG_STATE_CHANGE(oldActivity, newActivity, ...) \
	    XPCC_LOG_DEBUG << XPCC_FILE_INFO << "[sm:" ACTIVITY_LOG_NAME "] " << toStr(oldActivity) << " -> " << toStr(newActivity) << " ()" << xpcc::endl


/// Use at the beginning of an activity group
/// Must be used without index for `xpcc::NestedResumable`s and with index for `xpcc::Resumable`s.
#define ACTIVITY_GROUP_BEGIN(...) \
			RF_BEGIN(__VA_ARGS__); \
	        /* ACTIVITY_LOG_BEGIN(this->activity); */ \
			activity_switch: ATTRIBUTE_UNUSED; if (false)

/// Declares and activity
#define DECLARE_ACTIVITY(a) } else if(this->activity == a) {

/// Immidiately calls the declared activity. Additional parameter will be forwarded to ACTIVITY_LOG_STATE_CHANGE.
#define CALL_ACTIVITY(nextActivity, ...) do { \
	        ACTIVITY_LOG_STATE_CHANGE(this->activity, nextActivity, ##__VA_ARGS__); \
			this->activity = nextActivity; \
			goto activity_switch; } while(0)

/// Immidiately exits the group and sets the next activity to the declared one.
/// Additional parameter will be forwarded to ACTIVITY_LOG_STATE_CHANGE.
#define ACTIVITY_GROUP_EXIT(finalState, result, ...) do { \
			this->activity = finalState; \
	        /* ACTIVITY_LOG_EXIT(this->activity); */ \
			RF_RETURN(result); } while(0)

/// Ends the activity group
#define ACTIVITY_GROUP_END(result) \
			RF_END_RETURN(result);

#endif	// ROBOTS_ACTIVITIES_MACROS_HPP
