// coding: utf-8
/* Copyright (c) 2014, Roboterclub Aachen e. V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#ifndef XPCC__NRF24_CONFIG_HPP
#   error "Don't include this file directly, use 'nrf24_config.hpp' instead!"
#endif

#include "nrf24_config.hpp"
#include "nrf24_definitions.hpp"

#include <stdint.h>

// --------------------------------------------------------------------------------------------------------------------

template<typename Nrf24Phy>
void
xpcc::Nrf24Config<Nrf24Phy>::setMode(Mode mode)
{
	Nrf24Phy::clearInterrupt(InterruptFlag::ALL);

	if(mode == Mode::Rx)
	{
		XPCC_LOG_DEBUG << "Set mode Rx" << xpcc::endl;

		Nrf24Phy::flushRxFifo();
		Nrf24Phy::setBits(NrfRegister::CONFIG, Config::PRIM_RX);
	} else
	{
		XPCC_LOG_DEBUG << "Set mode Tx" << xpcc::endl;

		Nrf24Phy::clearInterrupt(Nrf24Phy::InterruptFlag::ALL);		// Not sure if needed
		Nrf24Phy::clearBits(NrfRegister::CONFIG, Config::PRIM_RX);

		// pulsing CE seems to be necessary to enter TX mode
		Nrf24Phy::pulseCe();
	}

	// don't go to standby
	Nrf24Phy::setCe();
}

// --------------------------------------------------------------------------------------------------------------------

template<typename Nrf24Phy>
typename xpcc::Nrf24Config<Nrf24Phy>::Mode
xpcc::Nrf24Config<Nrf24Phy>::getMode()
{
	uint8_t config = Nrf24Phy::readRegister(NrfRegister::CONFIG);

	if(config & static_cast<uint8_t>(Config::PRIM_RX)) {
		return Mode::Rx;
	} else {
		return Mode::Tx;
	}
}

// --------------------------------------------------------------------------------------------------------------------

template<typename Nrf24Phy>
void
xpcc::Nrf24Config<Nrf24Phy>::setSpeed(Speed speed)
{
	switch (speed)
	{
	case Speed::kBps250:
		Nrf24Phy::clearBits(NrfRegister::RF_SETUP, RfSetup::RF_DR_HIGH);
		Nrf24Phy::setBits  (NrfRegister::RF_SETUP, RfSetup::RF_DR_LOW);
		break;
	case Speed::MBps1:
		Nrf24Phy::clearBits(NrfRegister::RF_SETUP, RfSetup::RF_DR_LOW);
		Nrf24Phy::clearBits(NrfRegister::RF_SETUP, RfSetup::RF_DR_HIGH);
		break;
	case Speed::MBps2:
		Nrf24Phy::setBits  (NrfRegister::RF_SETUP, RfSetup::RF_DR_HIGH);
		Nrf24Phy::clearBits(NrfRegister::RF_SETUP, RfSetup::RF_DR_LOW);
		break;
	}
}

// --------------------------------------------------------------------------------------------------------------------

template<typename Nrf24Phy>
void xpcc::Nrf24Config<Nrf24Phy>::setCrc(Crc crc)
{
	if(crc == Crc::NoCrc)
	{
		Nrf24Phy::clearBits(NrfRegister::CONFIG, Config::EN_CRC);

	} else
	{
		Nrf24Phy::setBits(NrfRegister::CONFIG, Config::EN_CRC);

		if (crc == Crc::Crc1Byte)
		{
			Nrf24Phy::clearBits(NrfRegister::CONFIG, Config::CRC0);
		}
		else if (crc == Crc::Crc2Byte)
		{
			Nrf24Phy::setBits(NrfRegister::CONFIG, Config::CRC0);
		}
	}
}

// --------------------------------------------------------------------------------------------------------------------

template<typename Nrf24Phy>
void xpcc::Nrf24Config<Nrf24Phy>::setRfPower(RfPower power)
{
	uint8_t reg = Nrf24Phy::readRegister(NrfRegister::RF_SETUP);
	reg &= ~(static_cast<uint8_t>(RfSetup::RF_PWR));	// Clear bits
	reg |=  ((static_cast<uint8_t>(power)) << 1); 		// Set bits
	Nrf24Phy::writeRegister(NrfRegister::RF_SETUP, reg);
}

// --------------------------------------------------------------------------------------------------------------------

template<typename Nrf24Phy>
void xpcc::Nrf24Config<Nrf24Phy>::setAutoRetransmitDelay(AutoRetransmitDelay delay)
{
	uint8_t reg = Nrf24Phy::readRegister(NrfRegister::SETUP_RETR);
	reg &= ~(static_cast<uint8_t>(SetupRetr::ARD));		// Clear bits
	reg |=  ((static_cast<uint8_t>(delay)) << 4); 		// Set bits
	Nrf24Phy::writeRegister(NrfRegister::SETUP_RETR, reg);
}

// --------------------------------------------------------------------------------------------------------------------

template<typename Nrf24Phy>
void xpcc::Nrf24Config<Nrf24Phy>::setAutoRetransmitCount(AutoRetransmitCount count)
{
	uint8_t reg = Nrf24Phy::readRegister(NrfRegister::SETUP_RETR);
	reg &= ~(static_cast<uint8_t>(SetupRetr::ARC));		// Clear bits
	reg |=  (static_cast<uint8_t>(count)); 				// Set bits
	Nrf24Phy::writeRegister(NrfRegister::SETUP_RETR, reg);
}

// --------------------------------------------------------------------------------------------------------------------

template<typename Nrf24Phy>
void
xpcc::Nrf24Config<Nrf24Phy>::enablePipe(Pipe_t pipe, bool enableAutoAck)
{

	uint16_t payload_length = Nrf24Phy::getPayloadLength();

	NrfRegister_t reg = NrfRegister::RX_PW_P0;
	reg.value += pipe.value;

	/* Set payload width for pipe */
	Nrf24Phy::writeRegister(reg, payload_length);


	Flags_t pipe_flag = static_cast<Flags_t>(1 << pipe.value);

	/* Enable or disable auto acknowledgment for this pipe */
	if(enableAutoAck)
	{
		Nrf24Phy::setBits(NrfRegister::EN_AA, pipe_flag);
	} else
	{
		Nrf24Phy::clearBits(NrfRegister::EN_AA, pipe_flag);
	}

	/* enable pipe */
	Nrf24Phy::setBits(NrfRegister::EN_RX_ADDR, pipe_flag);
}

// --------------------------------------------------------------------------------------------------------------------

template<typename Nrf24Phy>
typename xpcc::Nrf24Config<Nrf24Phy>::Pipe_t
xpcc::Nrf24Config<Nrf24Phy>::getPayloadPipe()
{
	uint8_t status = Nrf24Phy::readStatus();

	return static_cast<Pipe_t>((status & (uint8_t)Status::RX_P_NO) >> 1);
}

// --------------------------------------------------------------------------------------------------------------------

template<typename Nrf24Phy>
bool
xpcc::Nrf24Config<Nrf24Phy>::channelBusy()
{
	// if we're transmitting we consider the channel as busy
	if(getMode() == Mode::Tx) {
		return true;
	}

	// We're in RX mode, so we need to pull CE low for RPD to be latched. CE is
	// assumed to be high during normal operation, so pulsing should be enough.
	// See section 6.4, p. 25
	Nrf24Phy::pulseCe();

	uint8_t rpd = Nrf24Phy::readRegister(NrfRegister::RPD);
	return static_cast<bool>(rpd);
}
