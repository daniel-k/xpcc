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

#include <stdlib.h>
#include <xpcc/processing.hpp>
#include <xpcc/debug/logger/style.hpp>
#include "r2mac.hpp"

template<typename Config>
typename xpcc::R2MAC<Config>::NodeList
xpcc::R2MAC<Config>::memberList;

template<typename Config>
typename xpcc::R2MAC<Config>::Role
xpcc::R2MAC<Config>::role =
        xpcc::R2MAC<Config>::Role::None;

template<typename Config>
typename xpcc::R2MAC<Config>::NodeAddress
xpcc::R2MAC<Config>::coordinatorAddress;

template<typename Config>
uint8_t
xpcc::R2MAC<Config>::memberCount = 0;

template<typename Config>
xpcc::Timestamp
xpcc::R2MAC<Config>::timeLastBeacon;

template<typename Config>
xpcc::Timestamp
xpcc::R2MAC<Config>::timestamp;

template<typename Config>
uint8_t
xpcc::R2MAC<Config>::ownDataSlot = 0;

template<typename Config>
typename xpcc::R2MAC<Config>::DataRXQueue
xpcc::R2MAC<Config>::dataRXQueue;

template<typename Config>
typename Config::L2Frame
xpcc::R2MAC<Config>::l2Frame;

template<typename Config>
typename xpcc::R2MAC<Config>::DataTXQueue
xpcc::R2MAC<Config>::dataTXQueue;

template<typename Config>
typename xpcc::R2MAC<Config>::AssociationQueue
xpcc::R2MAC<Config>::associationQueue;

template<typename Config>
typename xpcc::R2MAC<Config>::BeaconQueue
xpcc::R2MAC<Config>::beaconQueue;


template<typename Config>
void
xpcc::R2MAC<Config>::initialize(NetworkAddress network, NodeAddress address)
{
	// TODO: no abstraction of PHY, only works for Nrf24 here

	static_assert( (Config::Parameters::payloadLength + getFrameOverhead()) <=
	                Config::LinkLayer::Phy::getMaxPayload(),
	               "Payload length exceeds what Nrf24Phy can provide");

	static_assert((sizeof(typename Frames::Beacon) + getFrameOverhead()) <=
	              Config::LinkLayer::Phy::getMaxPayload(),
	              "Too many member nodes");

	Config::LinkLayer::Phy::initialize(phyPayloadSizeByte);
	Config::LinkLayer::initialize(network, address);

	// set these values to be able to calculate on-air frame size and time
	// at compile time
	Config::setSpeed(Config::Parameters::dataRate);
	Config::setAddressWidth(Config::Parameters::addressWidth);
	Config::setCrc(Config::Parameters::crcBytes);
	Config::setAutoRetransmitCount(Config::AutoRetransmitCount::Retransmit2);
	Config::setRfPower(Config::RfPower::Minus6dBm);

	// seed with own address, this at least guarantees a different seed on all
	// nodes in the network
	srand(address);

	R2MAC_LOG_DEBUG << "Data Slot duration: " << (timeDataSlotUs / 1000)
	                << " ms" << xpcc::endl;
	R2MAC_LOG_DEBUG << "Association Slot duration: "
	                << (timeAssociationSlotUs / 1000) << " ms"
	                << xpcc::endl;
}

template<typename Config>
void
xpcc::R2MAC<Config>::update()
{
	Config::LinkLayer::update();
	handlePackets();

	static xpcc::PeriodicTimer rateLimiter(200);

	if(rateLimiter.execute()) {
		R2MAC_LOG_INFO << "My role: "
		               << toStr(role) << xpcc::endl;
	}

	switch(role) {
	case Role::None:
		roleSelectionActivity.update();
		break;
	case Role::Coordinator:
		coordinatorActivity.update();
		break;
	case Role::Member:
		memberActivity.update();
		break;
	}
}

template<typename Config>
void
xpcc::R2MAC<Config>::waitUntilReadyToSend()
{
	while(not Config::LinkLayer::isReadyToSend()) {
		Config::LinkLayer::update();
	}
}

template<typename Config>
bool
xpcc::R2MAC<Config>::sendPacket(Packet& packet)
{
	packet.setType(Packet::Type::Data);

	const auto frame = reinterpret_cast<typename Config::L2Frame*>(&packet);
	return dataTXQueue.append(*frame);
}

template<typename Config>
bool
xpcc::R2MAC<Config>::getPacket(Packet& packet)
{
	if(dataRXQueue.isEmpty())
		return false;

	packet = Packet(dataRXQueue.getFront());
	dataRXQueue.removeFront();

	return true;
}

template<typename Config>
uint8_t
xpcc::R2MAC<Config>::getNeighbourList(NodeList& nodeList)
{
	nodeList = memberList;
	for(size_t i = 0; i < memberCount; i++) {
		// if our address is inside the list, we're a member so we can replace
		// ourself with the coordinator address
		if(nodeList[i] == getAddress()) {
			nodeList[i] = coordinatorAddress;
			break;
		}
	}
	return memberCount;
}

template<typename Config>
uint8_t
xpcc::R2MAC<Config>::getDataSlot(NodeAddress nodeAddress)
{
	for(uint8_t i = 0; i < memberCount; i++) {
		if (memberList[i] == nodeAddress) {
			return i + 1;
		}
	}

	return 0;
}

template<typename Config>
bool
xpcc::R2MAC<Config>::inMySlot()
{
	const xpcc::Timestamp endOwnSlotTimestamp = getStartOfOwnSlot() +
	        timeDataSlotUs;

	const xpcc::Timestamp now = Config::L2Clock::now();

	return (getStartOfOwnSlot() < now) and (now < endOwnSlotTimestamp);
}

template<typename Config>
typename xpcc::R2MAC<Config>::Packet::Type
xpcc::R2MAC<Config>::handlePackets(void)
{
	if(Config::LinkLayer::getPacket(l2Frame)) {

		const auto packet = reinterpret_cast<Packet*>(&l2Frame);
		const typename Packet::Type packetType = packet->getType();

		// filter packet by its destination address
		if ((packet->getDestination() != Config::LinkLayer::getBroadcastAddress()) and
		    (packet->getDestination() != Config::LinkLayer::getAddress())) {

			R2MAC_LOG_INFO << "Overheard " << Packet::toStr(packetType) << "("
			               << static_cast<uint8_t>(packetType) << ")"
			               << " frame from 0x" << xpcc::hex << packet->getSource()
			               << xpcc::ascii << " with 0x" << xpcc::hex
			               << packet->getDestination() << xpcc::ascii << xpcc::endl;

			// consider data packets as association requests if we're a
			// coordinator to reset lease timeouts
			// TODO: evaluate if this is needed
			if( (role == Role::Coordinator) and (packetType == Packet::Type::Data) ) {
				associationQueue.append(packet->getSource());
			}

			Config::LinkLayer::Phy::dumpRegisters();

		} else {

			R2MAC_LOG_INFO << "received a " << Packet::toStr(packetType) << " packet!" << xpcc::endl;

			// parse incoming packet
			switch(packetType) {
			case Packet::Type::Beacon: {

				const xpcc::Timestamp packetTimestamp = Config::LinkLayer::getFeedback().timestamp;
				const uint32_t timePassedUs = (packetTimestamp - timeLastBeacon).getTime();

				R2MAC_LOG_INFO << "Received new beacon frame from: 0x"
				               << xpcc::hex << packet->getSource() << xpcc::ascii
				               << " after " << timePassedUs / 1000 << " ms"
				               << xpcc::endl;

				// remember time of beacon and append
				timeLastBeacon = packetTimestamp;

				if(beaconQueue.isFull()) {
					R2MAC_LOG_ERROR << COLOR_RED << "Beacon queue is full!"
					                << COLOR_END << xpcc::endl;

					beaconQueue.removeFront();
				}

				beaconQueue.append(*packet);

				break;
			}
			case Packet::Type::AssociationRequest:
				// Append requesting node (packet source) address to the queue
				associationQueue.append(packet->getSource());

				R2MAC_LOG_INFO << "Received new association request from: 0x"
				               << xpcc::hex << packet->getSource() << xpcc::ascii
				               << xpcc::endl;
				break;
			case Packet::Type::Data:
				R2MAC_LOG_INFO << "Received new data packet from: 0x"
				               << xpcc::hex << packet->getSource() << xpcc::ascii
				               << xpcc::endl;

				if(not dataRXQueue.append(*packet)) {
					R2MAC_LOG_INFO << "Data RX queue is full, dropping packet"
					               << xpcc::endl;
				}
				break;
			case Packet::Type::None:
				break;
			}
		}

		return packetType;
	}

	return Packet::Type::None;
}
