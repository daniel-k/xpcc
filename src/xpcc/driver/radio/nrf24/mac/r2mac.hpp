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
#define XPCC_LOG_LEVEL xpcc::log::DISABLED

#define R2MAC_LOG_INFO	XPCC_LOG_INFO <<	"[r2mac:info] "
#define R2MAC_LOG_ERROR	XPCC_LOG_ERROR <<	"[r2mac:error] "

namespace xpcc
{

struct R2MACDefaultParameters
{
	/// Maximum number of associated member nodes (not counting coordinator)
	static constexpr int maxMembers = 15;

	/// Max payload that user application wants to transmit
	static constexpr int payloadLength = 29;

	/// Number of raw frames that fit into a data slot
	static constexpr int framesPerDataSlot = 4;

	/// Number of raw frames that fit into an association slot
	static constexpr int framesPerAssociationSlot = 1;

	/// Number of slots in association period
	static constexpr int associationSlots = 4; // maybe depend on maxNodes?

	/// Number of slots that a member will wait for a beacon frame
	static constexpr int maxMissedBeacons = 4;

	/// Node lease time
	static constexpr uint32_t timeNodeLeaseUs = 1000000;

	/// Probability in percent to become a coordinator if no super frame is
	/// received during role selection
	static constexpr int coordinatorElectionProbabilityPercent = 70;

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
	static constexpr int sendDataQueueSize = 3;

	/// Length of the association queue
	static constexpr int associationQueueSize = associationSlots;

	/// Length of the beacon queue (by initial design equal to 1)
	static constexpr int beaconQueueSize = 1;

};

/**
 * Realtime Robot MAC (R2MAC)
 *
 * @ingroup	nrf24
 * @author	Daniel Krebs
 */
template<typename Nrf24Data, class Parameters = R2MACDefaultParameters>
class R2MAC : private xpcc::Resumable<1>
{
public:
	using NetworkAddress = typename Nrf24Data::BaseAddress;
	using NodeAddress = typename Nrf24Data::Address;
	using NodeList = std::array<NodeAddress, Parameters::maxMembers>;
//	using Feedback = typename Nrf24Data::SendingState;
	using DataRXQueue = xpcc::BoundedDeque<typename Nrf24Data::Frame, Parameters::receivedDataQueueSize>;
	using DataTXQueue = xpcc::BoundedDeque<typename Nrf24Data::Frame, Parameters::sendDataQueueSize>;
	using BeaconQueue = xpcc::BoundedDeque<typename Nrf24Data::Frame, Parameters::beaconQueueSize>;
	using AssociationQueue = xpcc::BoundedDeque<NodeAddress, Parameters::associationQueueSize>;

	// convenience typedef
	using Config = typename Nrf24Data::Config;

	struct Feedback {
		bool success;
		typename Nrf24Data::Feedback state;
	};

	static constexpr uint8_t
	getPayloadLength()
	{ return Parameters::payloadLength; }

	// TODO: how to expose Header so that we can plug other layers on top in the
	//       same way?
	class xpcc_packed Frame
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
		uint8_t	payload[getPayloadLength()];


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

		const char*
		getTypeName() {
			switch(getType()) {
			case Type::Beacon:				return "Beacon";
			case Type::AssociationRequest:	return "AssociationRequest";
			case Type::Data:				return "Data";
			default:						return "invalid"; }
		}
	};

	static constexpr uint8_t
	getFrameOverhead()
	{ return Nrf24Data::getFrameOverhead() + sizeof (Frame::Header); }


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


//	class xpcc_packed Packet
//	{
//		Packet() :
//		    payloadLength(getPayloadLength()), feedback(nullptr) {}

//	// Buffer that will be sent in the end. User is not supposed to modify lower
//	// layer headers, so they are private.
//	private:
//		uint8_t headers[getFrameOverhead()];
//	public:
//		uint8_t payload[getPayloadLength()];
//	// -------------------------------------

//	public:
//		// Metadata of packet used for communication with user
//		uint8_t payloadLength;
//		NodeAddress sourceAddress;
//		NodeAddress destinationAddress;
//		Feedback* feedback;
//	};


public:
	static void
	initialize(NetworkAddress network, NodeAddress address);

	static void
	update();

	static uint8_t
	getNeighbourList(NodeList& nodeList);

	static bool
	sendPacket(Frames& packet);

	static bool
	getPacket(Frames& packet);

private:

	static constexpr uint32_t
	getSuperFrameDurationUs(uint8_t memberCount) {
		return (Parameters::associationSlots * timeAssociationSlotUs) +
		       ((memberCount + 1) * timeDataSlotUs) + // coordinator also has a slot
		       frameAirTimeUs; // beacon frame
	}

	/// Fixed payload size of every frame
	static constexpr uint8_t phyPayloadSizeByte = sizeof(Frame);

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
	static constexpr uint16_t timeGuardUs = 2 * timeSwitchUs;

	/// Duration of an association slot
	static constexpr uint32_t timeAssociationSlotUs =
	        (Parameters::framesPerAssociationSlot * frameAirTimeUs) + timeGuardUs;

	/// Duration of a data slot
	static constexpr uint32_t timeDataSlotUs =
	        (Parameters::framesPerDataSlot * frameAirTimeUs) + timeGuardUs;

	/// Worst case duration of a super frame, assuming a fully populated network
	static constexpr uint32_t timeMaxSuperFrameUs = getSuperFrameDurationUs(Parameters::maxMembers);

private:
	// TODO: replace by finer grained clock, xpcc::Clock is 1ms ticks by default
	using MicroSecondsClock = xpcc::Clock;
	using PeriodicTimerUs = xpcc::GenericPeriodicTimer<MicroSecondsClock>;
	using TimeoutUs = xpcc::GenericTimeout<MicroSecondsClock>;
	using NodeTimerList = std::array<TimeoutUs, Parameters::maxMembers>;

private:
	/// Check whether node is listed inside node list
	static uint8_t
	getDataSlot(NodeAddress nodeAddress);

	/// Enqueue received Nrf24 packets
	static FrameType
	handlePackets(void);

	/// Get a random value from (inclusive) interval [from, to]
	static uint32_t
	randomRange(uint32_t from, uint32_t to);

	/// Adopt network information from beacon frame
	static bool
	updateNetworkInfo(NodeAddress& coordinatorAddress, typename Frames::Beacon& beacon);

private:
	enum class
	Role
	{
		None,
		Coordinator,
		Member
	};

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

		Activity activity;
		TimeoutUs timeoutUs;
		typename Nrf24Data::Frame packet;
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
		Activity activity;
		TimeoutUs timeoutUs;
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
		Activity activity;
		TimeoutUs timeoutUs;
	};

private:
	static uint8_t memberCount;
	static NodeList memberList;
	static NodeTimerList memberLeaseTimeouts;
	static NodeAddress coordinatorAddress;
	static xpcc::Timestamp timeLastBeacon;
	static xpcc::Timestamp timestamp;
	static Role role;
	static RoleSelectionActivity roleSelectionActivity;
	static CoordinatorActivity coordinatorActivity;
	static MemberActivity memberActivity;
	static DataRXQueue dataRXQueue;
	static DataTXQueue dataTXQueue;
	static AssociationQueue associationQueue;
	static BeaconQueue beaconQueue;
	static uint8_t ownDataSlot;

	static typename Nrf24Data::Frame packet;
};

}

#include "r2mac_impl.hpp"
#include "r2mac_role_selection_impl.hpp"
#include "r2mac_coordinator_impl.hpp"
#include "r2mac_member_impl.hpp"

#endif // XPCC__NRF24_MAC_R2MAC_HPP
