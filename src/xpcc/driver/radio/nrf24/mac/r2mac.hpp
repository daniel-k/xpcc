// coding: utf-8
/* Copyright (c) 2017, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC__NRF24_MAC_R2MAC_HPP
#define XPCC__NRF24_MAC_R2MAC_HPP

#include <array>
#include <stdint.h>
#include <xpcc/debug/logger.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/container/deque.hpp>

#include <xpcc/driver/radio/nrf24/nrf24_config.hpp>
#include <xpcc/driver/radio/nrf24/nrf24_data.hpp>

#undef  XPCC_LOG_LEVEL
#define XPCC_LOG_LEVEL xpcc::log::WARNING

#define COLOR_RED        "\e[91m"
#define COLOR_GREEN      "\e[92m"
#define COLOR_YELLOW     "\e[93m"
#define COLOR_BLUE       "\e[94m"
#define COLOR_WHITE      "\e[97m"
#define COLOR_END        "\e[0m"

#define R2MAC_MODE_INDICATOR_START	((Config::currentMode == Config::Mode::Rx) ? COLOR_WHITE : COLOR_RED)
#define R2MAC_MODE_INDICATOR_END	(COLOR_END)

#define R2MAC_LOG_DEBUG	XPCC_LOG_DEBUG << R2MAC_MODE_INDICATOR_START << \
	                    "[r2mac:debug] " << R2MAC_MODE_INDICATOR_END

#define R2MAC_LOG_INFO	XPCC_LOG_INFO << R2MAC_MODE_INDICATOR_START << \
	                    "[r2mac:info] " << R2MAC_MODE_INDICATOR_END

#define R2MAC_LOG_ERROR XPCC_LOG_ERROR << R2MAC_MODE_INDICATOR_START << \
	                    "[r2mac:error] " << R2MAC_MODE_INDICATOR_END

namespace xpcc
{

struct R2MACDefaultParameters
{
	/// Maximum number of associated member nodes (not counting coordinator)
	static constexpr int maxMembers = 5;

	/// Max payload that user application wants to transmit
	static constexpr int payloadLength = 29;

	/// Number of raw frames that fit into a data slot
	static constexpr int framesPerDataSlot = 10;

	/// Number of raw frames that fit into an association slot
	static constexpr int framesPerAssociationSlot = 3;

	/// Number of slots in association period
	static constexpr int associationSlots = maxMembers / 2 + 1;

	/// Number of slots that a member will wait for a beacon frame
	static constexpr int maxMissedBeacons = 4;

	/// Node lease time
	static constexpr uint32_t timeNodeLeaseUs = 100 * 1000;

	/// Node update lease time (should be shorter than timeNodeLeaseUs)
	static constexpr uint32_t timeNodeLeaseUpdateUs = timeNodeLeaseUs / 2;

	/// Probability in percent to become a coordinator if no super frame is
	/// received during role selection
	static constexpr int coordinatorElectionProbabilityPercent = 50;

	/// Minimum number of association slots to listen for other beacon after
	/// self election to coordinator
	static constexpr int minSlotsAfterCoordinatorElection = 0;

	/// Maximum number of association slots to listen for other beacon after
	/// self election to coordinator
	static constexpr int maxSlotsAfterCoordinatorElection = 3;

	/// On-Air data rate
	static constexpr auto dataRate = xpcc::Nrf24ConfigParameters::Speed::MBps1;

	/// Number of bytes that constitute an address
	static constexpr auto addressWidth = xpcc::Nrf24ConfigParameters::AddressWidth::Byte5;

	/// How many bytes will be used for CRC
	static constexpr auto crcBytes = xpcc::Nrf24ConfigParameters::Crc::Crc2Byte;

	/// Length of the received data queue
	static constexpr int receivedDataQueueSize = 3;

	/// Length of the transmit data queue
	static constexpr int sendDataQueueSize = 32;
};

/**
 * Realtime Robot MAC (R2MAC)
 *
 * @ingroup	nrf24
 * @author	Daniel Krebs
 * @author  Tomasz Chyrowicz
 */
template<typename Nrf24Data, class Parameters = R2MACDefaultParameters>
class R2MAC : private xpcc::Resumable<1>
{
public:
	using Feedback = typename Nrf24Data::SendingFeedback;
	using NetworkAddress = typename Nrf24Data::BaseAddress;
	using NodeAddress = typename Nrf24Data::Address;
	using NodeList = std::array<NodeAddress, Parameters::maxMembers>;

	class xpcc_packed Packet
	{
		// R2MAC needs to access private interfaces
		friend R2MAC;


	// ------ Types ------
	public:
		enum class
		Type : uint8_t
		{
			Beacon = 0x01,
			AssociationRequest = 0x02,
			Data = 0x03,
			None = 0xff
		};

		static const char*
		toStr(Type type) {
			switch(type) {
			case Type::Beacon:				return "Beacon";
			case Type::AssociationRequest:	return "AssociationRequest";
			case Type::Data:				return "Data";
			case Type::None:				return "None";
			default:						return "Invalid"; }
		}

	private:
		struct xpcc_packed Header
		{
			Type type;
		};

		using HeaderBelow = typename Nrf24Data::Header;


	// ------ Data ------
	/// Memory layout is such that an instance of this class can be piped into
	/// the PHY without further alterations.
	private:
		HeaderBelow headerBelow;
		Header	header;
	public:
		/// User will put data here
		uint8_t	payload[Nrf24Data::Packet::getPayloadLength() - sizeof(Header)];


	// ------ Functions ------
	public:
		xpcc_always_inline void
		setDestination(NodeAddress dest)
		{ headerBelow.dest = dest; }

		xpcc_always_inline NodeAddress
		getSource()
		{ return headerBelow.src; }

		xpcc_always_inline NodeAddress
		getDestination()
		{ return headerBelow.dest; }

	private:
		xpcc_always_inline void
		setType(Type type)
		{ header.type = type; }

		xpcc_always_inline Type
		getType()
		{ return header.type; }
	};

public:
	/// Initialization of R2MAC, has to be called prior to usage
	static void
	initialize(NetworkAddress network, NodeAddress address);

	/// Call as often as possible
	static void
	update();

	static NodeAddress
	getAddress();

	static uint8_t
	getNeighbourList(NodeList& nodeList);

	static bool
	sendPacket(Packet& packet);

	static bool
	getPacket(Packet& packet);

	static Feedback
	getFeedback()
	{ return Feedback::DontKnow; }

	static xpcc_always_inline bool
	isAssociated()
	{ return ((ownDataSlot != 0) or (role == Role::Coordinator)); }

	static constexpr uint8_t
	getFrameOverhead()
	{ return Nrf24Data::getFrameOverhead() + sizeof(typename Packet::Header); }

	static constexpr uint8_t
	getPayloadLength()
	{ return Parameters::payloadLength; }

private:
	using Config = typename Nrf24Data::Config;
	using Nrf24DataPacket = typename Nrf24Data::Packet;

	static constexpr uint32_t
	getSuperFrameDurationUs(uint8_t memberCount) {
		return timeAssociationPeriodUs +
		       ((memberCount + 1) * timeDataSlotUs) + // coordinator also has a slot
		       frameAirTimeUs; // beacon frame
	}

	/// Fixed payload size of every frame
	static constexpr uint8_t phyPayloadSizeByte = sizeof(Packet);

	/// On-air frame size in bits
	static constexpr uint16_t frameAirSizeBits =
	        8 * (1 // Preamble
	             + Config::toNum(Parameters::addressWidth)
	             + phyPayloadSizeByte
	             + Config::toNum(Parameters::crcBytes))
	        + 9; // Packet Control Field

	/// On-air frame time in microseconds
	static constexpr uint16_t frameAirTimeUs =
	        (1000000UL * frameAirSizeBits) / static_cast<uint32_t>(Parameters::dataRate);

	/// Switching time between Rx,Tx and Standby (all the same) in microseconds
	static constexpr uint8_t timeSwitchUs = 130;

	/// Guard interval at end of each slot
	static constexpr uint16_t timeGuardUs = 4 * timeSwitchUs;

	/// Duration of an association slot
	static constexpr uint32_t timeAssociationSlotUs =
	        (Parameters::framesPerAssociationSlot * frameAirTimeUs) + timeGuardUs;

	/// Duration of whole association period
	static constexpr uint32_t timeAssociationPeriodUs =
	        timeAssociationSlotUs * Parameters::associationSlots;

	/// Time of a data slot when transmission is allowed
	static constexpr uint32_t timeTransmissionUs =
	        Parameters::framesPerDataSlot * frameAirTimeUs;

	/// Duration of a data slot
	static constexpr uint32_t timeDataSlotUs = timeTransmissionUs + timeGuardUs;

	/// Worst case duration of a super frame, assuming a fully populated network
	static constexpr uint32_t timeMaxSuperFrameUs =
	        getSuperFrameDurationUs(Parameters::maxMembers);

	/// Length of the association queue
	static constexpr int associationQueueSize = Parameters::associationSlots;

	/// Length of the beacon queue (by initial design equal to 1)
	static constexpr int beaconQueueSize = 1;

private:
	class Frames
	{
	public:
		struct xpcc_packed Beacon {
			uint8_t memberCount;
			uint8_t members[Parameters::maxMembers];
		};

		// AssociationRequest currently has no payload, hence no definition here

		struct xpcc_packed Data {
			uint8_t payload[getPayloadLength()];
		};
	};

	using DataRXQueue = xpcc::BoundedDeque<Packet, Parameters::receivedDataQueueSize>;
	using DataTXQueue = xpcc::BoundedDeque<Nrf24DataPacket, Parameters::sendDataQueueSize>;
	using BeaconQueue = xpcc::BoundedDeque<Packet, beaconQueueSize>;
	using AssociationQueue = xpcc::BoundedDeque<NodeAddress, associationQueueSize>;

private:
	using MicroSecondsClock = typename Nrf24Data::ClockLower;
	using PeriodicTimerUs = xpcc::GenericPeriodicTimer<MicroSecondsClock>;
	using TimeoutUs = xpcc::GenericTimeout<MicroSecondsClock>;
	using NodeTimerList = std::array<TimeoutUs, Parameters::maxMembers>;

private:
	/// Check whether node is listed inside node list
	static uint8_t
	getDataSlot(NodeAddress nodeAddress);

	/// Enqueue received Nrf24 packets
	static typename Packet::Type
	handlePackets(void);

	/// Get a random value from (inclusive) interval [from, to]
	static uint32_t
	randomRange(uint32_t from, uint32_t to);

	/// Adopt network information from beacon frame
	static bool
	updateNetworkInfo(NodeAddress coordinatorAddress, typename Frames::Beacon& beacon);

	static bool
	inMySlot();

	static xpcc_always_inline xpcc::Timestamp
	    getStartOfOwnSlot()
	{ return timeLastBeacon + timeAssociationPeriodUs + (ownDataSlot * timeDataSlotUs); }

private:
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

	class RoleSelectionActivity : private xpcc::Resumable<1>
	{
	public:
		RoleSelectionActivity()
		{ initialize(); }

		void
		initialize();

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
		TimeoutUs timeoutUs;
	};

	class CoordinatorActivity : private xpcc::Resumable<1>
	{
	public:
		CoordinatorActivity()
		{ initialize(); }

		void
		initialize();

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
		TimeoutUs timeoutUs;
		NodeTimerList memberLeaseTimeouts;
		xpcc::Timestamp targetTimestamp;
	};

	class MemberActivity : private xpcc::Resumable<1>
	{
	public:
		MemberActivity()
		{ initialize(); }

		void
		initialize();

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
		TimeoutUs timeoutUs;
		TimeoutUs leaseTimeoutUs;
		int32_t associationSlot;
		xpcc::Timestamp targetTimestamp;

	};

private:
	static NodeAddress coordinatorAddress;
	static uint8_t memberCount;
	static NodeList memberList;
	static uint8_t ownDataSlot;

	static xpcc::Timestamp timeLastBeacon;
	static xpcc::Timestamp timestamp;

	static DataRXQueue dataRXQueue;
	static DataTXQueue dataTXQueue;
	static AssociationQueue associationQueue;
	static BeaconQueue beaconQueue;

	static Role role;
	static RoleSelectionActivity roleSelectionActivity;
	static CoordinatorActivity coordinatorActivity;
	static MemberActivity memberActivity;
};

}

#include "r2mac_impl.hpp"
#include "r2mac_role_selection_impl.hpp"
#include "r2mac_coordinator_impl.hpp"
#include "r2mac_member_impl.hpp"

#endif // XPCC__NRF24_MAC_R2MAC_HPP
