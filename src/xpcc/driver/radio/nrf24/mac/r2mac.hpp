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

#include "r2mac_packet.hpp"
#include "r2mac_roles.hpp"

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
	static constexpr int framesPerAssociationSlot = 4;

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


template<typename Nrf24Data, typename Parameters_ = R2MACDefaultParameters>
class Config {

	using Parameters				= Parameters_;
	using LinkLayer					= Nrf24Data;

	// interface that needs to be provided by LinkLayer
	using L2Clock			= typename LinkLayer::ClockLower;
	using L2Address			= typename LinkLayer::Address;
	using L2NetworkAddress	= typename LinkLayer::BaseAddress;
	using L2Header			= typename LinkLayer::Header;
	using L2Frame			= typename LinkLayer::Packet;

	// time related
	using PeriodicTimerUs	= xpcc::GenericPeriodicTimer<L2Clock>;
	using TimeoutUs			= xpcc::GenericTimeout<L2Clock>;
	using MemberTimeoutList	= std::array<TimeoutUs, Parameters::maxMembers>;
};


/**
 * Realtime Robot MAC (R2MAC)
 *
 * @ingroup	nrf24
 * @author	Daniel Krebs
 * @author  Tomasz Chyrowicz
 */
template<typename Config>
class R2MAC :
        public R2MACPacket<Config>,
        public R2MACRoles<Config>,
        private xpcc::Resumable<1>
{
public:

	using NetworkAddress	= typename Config::L2NetworkAddress;
	using NodeAddress		= typename Config::L2Address;

	using Feedback			= typename Config::L2::SendingFeedback;
	using NodeList			= std::array<NodeAddress, Config::Parameters::maxMembers>;

	// dependent types
	using typename xpcc::R2MACPacket<Config>::Packet;
	using typename xpcc::R2MACRoles<Config>::Role;
	using typename xpcc::R2MACRoles<Config>::Activities;


public:
	/// Initialization of R2MAC, has to be called prior to usage
	static void
	initialize(NetworkAddress network, NodeAddress address);

	/// Call as often as possible
	static void
	update();

	static uint8_t
	getNeighbourList(NodeList& nodeList);

	static bool
	sendPacket(Packet& packet);

	static bool
	getPacket(Packet& packet);

	static xpcc_always_inline NodeAddress
	getAddress()
	{ return Config::LinkLayer::getAddress(); }

	static xpcc_always_inline bool
	isAssociated()
	{ return ((ownDataSlot != 0) or (role == Role::Coordinator)); }

	static constexpr uint8_t
	getFrameOverhead()
	{ return Packet::getFrameOverhead(); }

	static constexpr uint8_t
	getPayloadLength()
	{ return Config::Parameters::payloadLength; }

private:

	static constexpr uint32_t
	getSuperFrameDurationUs(uint8_t memberCount)
	{ return timeAssociationPeriodUs + ((memberCount + 1) * timeDataSlotUs); }

	/// Fixed payload size of every frame
	static constexpr uint8_t phyPayloadSizeByte = sizeof(Packet);

	/// On-air frame time in microseconds
	static constexpr uint16_t frameAirTimeUs =
	        frameAirTimeUs(phyPayloadSizeByte,
	                       Config::Parameters::addressWidth,
	                       Config::Parameters::crcBytes,
	                       Config::Parameters::dataRate);

	/// Switching time between Rx,Tx and Standby (all the same) in microseconds
	static constexpr uint8_t timeSwitchUs = 130;

	/// Guard interval at end of each slot
	static constexpr uint16_t timeGuardUs = 4 * timeSwitchUs;

	/// Time of an association slot when transmission is allowed
	static constexpr uint32_t timeAssociationTransmissionUs =
	        Config::Parameters::framesPerAssociationSlot * frameAirTimeUs;

	/// Duration of an association slot
	static constexpr uint32_t timeAssociationSlotUs =
	        timeAssociationTransmissionUs + timeGuardUs;

	/// Duration of whole association period
	static constexpr uint32_t timeAssociationPeriodUs =
	        timeAssociationSlotUs * Config::Parameters::associationSlots;

	/// Time of a data slot when transmission is allowed
	static constexpr uint32_t timeDataTransmissionUs =
	        Config::Parameters::framesPerDataSlot * frameAirTimeUs;

	/// Duration of a data slot
	static constexpr uint32_t timeDataSlotUs =
	        timeDataTransmissionUs + timeGuardUs;

	/// Worst case duration of a super frame, assuming a fully populated network
	static constexpr uint32_t timeMaxSuperFrameUs =
	        getSuperFrameDurationUs(Config::Parameters::maxMembers);

	/// Length of the association queue
	static constexpr int associationQueueSize =
	        Config::Parameters::associationSlots;

	/// Length of the beacon queue (by initial design equal to 1)
	static constexpr int beaconQueueSize = 1;

private:
	class Frames
	{
	public:
		struct xpcc_packed Beacon {
			uint8_t memberCount;
			uint8_t members[Config::Parameters::maxMembers];
		};

		// AssociationRequest currently has no payload, hence no definition here

		struct xpcc_packed Data {
			uint8_t payload[getPayloadLength()];
		};
	};

	using DataRXQueue = xpcc::BoundedDeque<Packet, Config::Parameters::receivedDataQueueSize>;
	using DataTXQueue = xpcc::BoundedDeque<typename Config::L2Frame, Config::Parameters::sendDataQueueSize>;
	using BeaconQueue = xpcc::BoundedDeque<Packet, beaconQueueSize>;
	using AssociationQueue = xpcc::BoundedDeque<NodeAddress, associationQueueSize>;

private:
	/// Check whether node is listed inside node list
	static uint8_t
	getDataSlot(NodeAddress nodeAddress);

	/// Enqueue received Nrf24 packets
	static typename Packet::Type
	handlePackets(void);

	static bool
	inMySlot();

	static xpcc_always_inline void
	clearAssociation()
	{ ownDataSlot = 0; /* memberCount = 0; */ }

	/// Get a random value from (inclusive) interval [from, to]
	static xpcc_always_inline uint32_t
	randomRange(uint32_t from, uint32_t to)
	{ return (from + (rand() % (to - from + 1))); }

	static xpcc_always_inline xpcc::Timestamp
	getStartOfOwnSlot()
	{ return timeLastBeacon + timeAssociationPeriodUs + (ownDataSlot * timeDataSlotUs); }

	static void
	waitUntilReadyToSend();


private:
	static NodeAddress coordinatorAddress;
	static uint8_t memberCount;
	static NodeList memberList;
	static uint8_t ownDataSlot;

	static xpcc::Timestamp timeLastBeacon;
	static xpcc::Timestamp timestamp;

	static typename Config::L2Frame l2Frame;

	static DataRXQueue dataRXQueue;
	static DataTXQueue dataTXQueue;
	static AssociationQueue associationQueue;
	static BeaconQueue beaconQueue;

	static Role role;
	static typename Activities::RoleSelectionActivity roleSelectionActivity;
	static typename Activities::CoordinatorActivity coordinatorActivity;
	static typename Activities::MemberActivity memberActivity;
};

}

#include "r2mac_impl.hpp"
#include "r2mac_role_selection_impl.hpp"
#include "r2mac_coordinator_impl.hpp"
#include "r2mac_member_impl.hpp"

#endif // XPCC__NRF24_MAC_R2MAC_HPP
