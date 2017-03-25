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

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::NodeList
xpcc::R2MAC<Nrf24Data, Parameters>::memberList;

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::Role
xpcc::R2MAC<Nrf24Data, Parameters>::role =
        xpcc::R2MAC<Nrf24Data, Parameters>::Role::None;

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::NodeAddress
xpcc::R2MAC<Nrf24Data, Parameters>::coordinatorAddress;

template<typename Nrf24Data, class Parameters>
uint8_t
xpcc::R2MAC<Nrf24Data, Parameters>::memberCount = 0;

template<typename Nrf24Data, class Parameters>
xpcc::Timestamp
xpcc::R2MAC<Nrf24Data, Parameters>::timeLastBeacon;

template<typename Nrf24Data, class Parameters>
xpcc::Timestamp
xpcc::R2MAC<Nrf24Data, Parameters>::timestamp;

template<typename Nrf24Data, class Parameters>
uint8_t
xpcc::R2MAC<Nrf24Data, Parameters>::ownDataSlot = 0;

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::DataRXQueue
xpcc::R2MAC<Nrf24Data, Parameters>::dataRXQueue;

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::DataTXQueue
xpcc::R2MAC<Nrf24Data, Parameters>::dataTXQueue;

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::AssociationQueue
xpcc::R2MAC<Nrf24Data, Parameters>::associationQueue;

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::BeaconQueue
xpcc::R2MAC<Nrf24Data, Parameters>::beaconQueue;



template<typename Nrf24Data, class Parameters>
void
xpcc::R2MAC<Nrf24Data, Parameters>::initialize(NetworkAddress network, NodeAddress address)
{
	static_assert( (Parameters::payloadLength + getFrameOverhead()) <=
	                Nrf24Data::Phy::getMaxPayload(),
	               "Payload length exceeds what Nrf24Phy can provide");

	static_assert((sizeof(typename Frames::Beacon) + getFrameOverhead()) <=
	              Nrf24Data::Phy::getMaxPayload(),
	              "Too many member nodes");

	Nrf24Data::Phy::initialize(phyPayloadSizeByte);
	Nrf24Data::initialize(network, address);

	// set these values to be able to calculate on-air frame size and time
	// at compile time
	Config::setSpeed(Parameters::dataRate);
	Config::setAddressWidth(Parameters::addressWidth);
	Config::setCrc(Parameters::crcBytes);
	Config::setAutoRetransmitCount(Config::AutoRetransmitCount::Retransmit1);

	// TODO: we still need some entropy/seed for the PRNG later
}

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::NodeAddress
xpcc::R2MAC<Nrf24Data, Parameters>::getAddress()
{
	return Nrf24Data::getAddress();
}

template<typename Nrf24Data, class Parameters>
void
xpcc::R2MAC<Nrf24Data, Parameters>::update()
{
	Nrf24Data::update();
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

template<typename Nrf24Data, class Parameters>
bool
xpcc::R2MAC<Nrf24Data, Parameters>::sendPacket(Packet& packet)
{
	packet.setType(Packet::Type::Data);

	const auto packetNrfData = reinterpret_cast<Nrf24DataPacket*>(&packet);
	return dataTXQueue.append(*packetNrfData);
}

template<typename Nrf24Data, class Parameters>
bool
xpcc::R2MAC<Nrf24Data, Parameters>::getPacket(Packet& packet)
{
	if(dataRXQueue.isEmpty())
		return false;

	packet = Packet(dataRXQueue.getFront());
	dataRXQueue.removeFront();

	return true;
}

template<typename Nrf24Data, class Parameters>
uint8_t
xpcc::R2MAC<Nrf24Data, Parameters>::getNeighbourList(NodeList& nodeList)
{
	memcpy(nodeList, memberList, memberCount * sizeof(NodeAddress));
	return memberCount;
}

template<typename Nrf24Data, class Parameters>
uint8_t
xpcc::R2MAC<Nrf24Data, Parameters>::getDataSlot(NodeAddress nodeAddress)
{
	for(uint8_t i = 0; i < memberCount; i++) {
		if (memberList[i] == nodeAddress) {
			return i + 1;
		}
	}

	return 0;
}

template<typename Nrf24Data, class Parameters>
typename xpcc::R2MAC<Nrf24Data, Parameters>::Packet::Type
xpcc::R2MAC<Nrf24Data, Parameters>::handlePackets(void)
{
	Nrf24DataPacket packetNrf24Data;

	if(Nrf24Data::getPacket(packetNrf24Data)) {

		const auto packet = reinterpret_cast<Packet*>(&packetNrf24Data);
		const typename Packet::Type packetType = packet->getType();

		// filter packet by its destination address
//		if ((packet->getDestination() != Nrf24Data::getBroadcastAddress()) and
//		    (packet->getDestination() != Nrf24Data::getAddress())) {

//			R2MAC_LOG_INFO << "Overheard " << packet->getTypeName()
//			               << " frame from 0x" << xpcc::hex << packet->getSource()
//			               << xpcc::ascii << " with 0x" << xpcc::hex
//			               << packet->getDestination() << xpcc::ascii << xpcc::endl;

//			// consider data packets as association requests if we're a
//			// coordinator to reset lease timeouts
//			// TODO: evaluate if this is needed
//			if( (role == Role::Coordinator) and (packetType == Packet::Type::Data) ) {
//				associationQueue.append(packet->getSource());
//			}
//		} else {

		    R2MAC_LOG_INFO << "received a " << Packet::toStr(packetType) << " packet!" << xpcc::endl;

			// parse incoming packet
			switch(packetType) {
			case Packet::Type::Beacon: {

				const xpcc::Timestamp packetTimestamp = Nrf24Data::getFeedback().timestamp;
				const uint32_t timePassedUs = (packetTimestamp - timeLastBeacon).getTime();

				R2MAC_LOG_INFO << "Received new beacon frame from: 0x"
				               << xpcc::hex << packet->getSource() << xpcc::ascii
				               << " after " << timePassedUs / 1000 << " ms"
				               << xpcc::endl;

				// remember time of beacon and append
				timeLastBeacon = packetTimestamp;
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
//		}

		return packetType;
	}

	return Packet::Type::None;
}

template<typename Nrf24Data, class Parameters>
uint32_t
xpcc::R2MAC<Nrf24Data, Parameters>::randomRange(uint32_t from, uint32_t to)
{
	return from + (rand() % (to - from + 1));
}


template<typename Nrf24Data, class Parameters>
bool
xpcc::R2MAC<Nrf24Data, Parameters>::updateNetworkInfo(NodeAddress coordinatorAddr,
                                                typename Frames::Beacon& beacon)
{
	NodeAddress myAddress = getAddress();
	coordinatorAddress = coordinatorAddr;

	memberCount = beacon.memberCount;
	for(uint8_t i = 0; i < memberCount; i++) {
		memberList[i] = static_cast<NodeAddress>(beacon.members[i]);

		if(memberList[i] == myAddress) {
			ownDataSlot = i + 1;
		}
	}

	return true;
}
