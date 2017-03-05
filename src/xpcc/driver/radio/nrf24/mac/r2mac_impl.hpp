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

	switch(role) {
	case Role::None:
		roleSelectionActivity.update();
		break;
	case Role::Coordinator:
		coordinatorActivity.update()();
		break;
	case Role::Member:
		break;
	}
}

template<typename Nrf24Data, class Parameters>
bool
xpcc::R2MAC<Nrf24Data, Parameters>::sendPacket(Frames& packet)
{
	return dataTXQueue.append(packet);
}

template<typename Nrf24Data, class Parameters>
bool
xpcc::R2MAC<Nrf24Data, Parameters>::getPacket(Frames& packet)
{
	if(dataRXQueue.isEmpty())
		return false;

	packet = dataRXQueue.getFront();
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
typename xpcc::R2MAC<Nrf24Data, Parameters>::FrameType
xpcc::R2MAC<Nrf24Data, Parameters>::handlePackets(void)
{
	if(Nrf24Data::getPacket(packet)) {
		const FrameType frameType = Frames::getType(packet);

		// filter packet by its destination address
		if ((packet.dest != Nrf24Data::getBroadcastAddress()) and
			(packet.dest != Nrf24Data::getAddress())) {

			R2MAC_LOG_INFO << "Overheard " << Frames::getName(packet)
			               << " frame from 0x" << xpcc::hex << packet.src
			               << xpcc::ascii << " with 0x" << xpcc::hex
			               << packet.dest << xpcc::ascii << xpcc::endl;

			// consider data packets as association requests if we're a
			// coordinator to reset lease timeouts
			// TODO: evaluate if this is needed
			if( (role == Role::Coordinator) and (frameType == FrameType::Data) ) {
				associationQueue.append(packet.src);
			}
		} else {
			// parse incoming packet
			switch(Frames::getType(packet)) {
			case FrameType::Beacon: {
				timeLastBeacon = packet.timestamp;
				beaconQueue.append(packet);

				const int32_t timePassedUs = packet.timestamp - timeLastBeacon;
				R2MAC_LOG_INFO << "Received new beacon frame from: 0x"
				               << xpcc::hex << packet.src << xpcc::ascii
				               << " after: " << timePassedUs / 1000 << " ms"
				               << xpcc::endl;
				break;
			}
			case FrameType::AssociationRequest:
				// Append requesting node (packet source) address to the queue
				associationQueue.append(packet.src);

				R2MAC_LOG_INFO << "Received new association request from: 0x"
				               << xpcc::hex << packet.src << xpcc::ascii
				               << xpcc::endl;
				break;
			case FrameType::Data:
				R2MAC_LOG_INFO << "Received new data packet from: 0x"
				               << xpcc::hex << packet.src << xpcc::ascii
				               << xpcc::endl;

				if(not dataRXQueue.append(packet)) {
					R2MAC_LOG_INFO << "Data RX queue is full, dropping packet"
					               << xpcc::endl;
				}
				break;
			}
		}

		return frameType;
	}

	return FrameType::None;
}

template<typename Nrf24Data, class Parameters>
uint32_t
xpcc::R2MAC<Nrf24Data, Parameters>::randomRange(uint32_t from, uint32_t to)
{
	return from + (rand() % (to - from + 1));
}


template<typename Nrf24Data, class Parameters>
bool
xpcc::R2MAC<Nrf24Data, Parameters>::updateNetworkInfo(NodeAddress& coordinatorAddress,
                                                typename Frames::Beacon& beacon)
{
	NodeAddress myAddress = getAddress();
	coordinatorAddress = coordinatorAddress;

	memberCount = beacon.memberCount;
	for(uint8_t i = 0; i < memberCount; i++) {
		memberList[i] = static_cast<NodeAddress>(beacon.members[i]);

		if(memberList[i] == myAddress) {
			ownDataSlot = i + 1;
		}
	}

	return true;
}
