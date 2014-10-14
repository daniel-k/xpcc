// coding: utf-8
/* Copyright (c) 2013, Roboterclub Aachen e.V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
// ----------------------------------------------------------------------------

#include <xpcc/architecture/platform.hpp>

using namespace xpcc::atmega;

typedef GpioOutputB0 Led;
typedef GpioOutputB1 Led2;

typedef xpcc::GpioInverted< Led2 > LedInverted;

typedef GpioPort<GpioOutputD0, 8> Data;
typedef GpioPort<GpioOutputD2, 5> Data3;
typedef xpcc::SoftwareGpioPort<GpioC1, GpioC4, GpioB6, GpioB3, GpioB5> Data2;

static_assert(Data::width == 8, "Data::width is not 8");
static_assert(Data2::width == 5, "Data2::width is not 5");

typedef SpiSimpleMaster SPI;
typedef UartSpiSimpleMaster0 SPI2;
typedef Uart0 UART;
typedef xpcc::avr::SystemClock systemClock;

MAIN_FUNCTION
{
	Led::setOutput();
	Led::set();
	Data::setOutput();
	volatile uint8_t datat = 0x0f;
	Data::write(datat);
	Data3::write(datat);

	Data2::setOutput();
	datat = 0x1e;
	Data2::write(datat);

	LedInverted::setOutput();
	LedInverted::reset();

	GpioD0::connect(UART::Rx);
	GpioD1::connect(UART::Tx);

	SPI::initialize<systemClock, 7372800>();
	SPI2::initialize<systemClock, 7372800>();
	UART::initialize<systemClock, 225>();

	while (1)
	{
	}
}
