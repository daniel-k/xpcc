// coding: utf-8
/* Copyright (c) 2016, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC__NRF24_RAVE_MAC_HPP
#define XPCC__NRF24_RAVE_MAC_HPP

#include <stdint.h>
#include <xpcc/debug/logger.hpp>
#include <xpcc/processing.hpp>

#include "nrf24_data.hpp"

#undef  XPCC_LOG_LEVEL
#define XPCC_LOG_LEVEL xpcc::log::DISABLED

namespace xpcc
{

/**
 * Random Access Virtual carrier sensE MAC
 *
 * This basic MAC protocol implements a virtual carrier sensing mechanism using
 * Ready-to-Send (RTS) and Clear-to-Send (CTS) packets as described in MACA [1].
 *
 * [1] http://www.arrl.org/files/file/History/History%20of%20QST%20Volume%201%20-%20Technology/NetCon9-Karn.pdf
 * @ingroup	nrf24
 * @author	Daniel Krebs
 */
template<typename Nrf24Data>
class RaveMAC
{
public:
	static constexpr int maxCtsDelayMs = 2;
	static constexpr int maxRetransmit = 1;

public:
	void
	update();

private:

	enum class
	PacketType : uint8_t
	{
		RTS,
		CTS,
		DATA
	};

	struct ATTRIBUTE_PACKED RTS {
		RTS() : type(PacketType::RTS) {}
		PacketType type;
		uint8_t dataLength;
		uint8_t sequenceNumber;
		typename Nrf24Data::Address destination;
	};

	struct ATTRIBUTE_PACKED CTS {
		CTS() : type(PacketType::CTS) {}
		PacketType type;
		uint8_t dataLength;
		uint8_t sequenceNumber;
		typename Nrf24Data::Address destination;
	};

	struct ATTRIBUTE_PACKED DATA {
		DATA() : type(PacketType::DATA) {}
		PacketType type;
		typename Nrf24Data::Packet::Payload payload;
	};
};

}

#endif // XPCC__NRF24_RAVE_MAC_HPP
