/* Copyright (c) 2016, Roboterclub Aachen e.V.
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 * ------------------------------------------------------------------------ */

//
// NUCLEO-F401RE
// Nucleo kit for STM32F401RE
//

#ifndef XPCC_STM32_NUCLEO_F401RE_HPP
#define XPCC_STM32_NUCLEO_F401RE_HPP

#include <xpcc/architecture/platform.hpp>
#include <xpcc/debug/logger.hpp>
#define XPCC_BOARD_HAS_LOGGER

using namespace xpcc::stm32;


namespace Board
{

/// STM32F401RE running at 84MHz generated from the internal 16MHz crystal
// Dummy clock for devices
struct systemClock {
	static constexpr uint32_t Frequency = 84 * MHz1;
	static constexpr uint32_t Ahb = Frequency;
	static constexpr uint32_t Apb1 = Frequency / 2;
	static constexpr uint32_t Apb2 = Frequency;

	static constexpr uint32_t Adc1   = Apb2;

	static constexpr uint32_t Spi1   = Apb2;
	static constexpr uint32_t Spi2   = Apb1;
	static constexpr uint32_t Spi3   = Apb1;
	static constexpr uint32_t Spi4   = Apb2;

	static constexpr uint32_t Usart1 = Apb2;
	static constexpr uint32_t Usart2 = Apb1;
	static constexpr uint32_t Usart6 = Apb2;

	static constexpr uint32_t I2c1   = Apb1;
	static constexpr uint32_t I2c2   = Apb1;
	static constexpr uint32_t I2c3   = Apb1;

	static constexpr uint32_t Apb1Timer = Apb1 * 2;
	static constexpr uint32_t Apb2Timer = Apb2 * 1;
	static constexpr uint32_t Timer1  = Apb2Timer;
	static constexpr uint32_t Timer2  = Apb1Timer;
	static constexpr uint32_t Timer3  = Apb1Timer;
	static constexpr uint32_t Timer4  = Apb1Timer;
	static constexpr uint32_t Timer5  = Apb1Timer;
	static constexpr uint32_t Timer8  = Apb2Timer;
	static constexpr uint32_t Timer9  = Apb2Timer;
	static constexpr uint32_t Timer10 = Apb2Timer;
	static constexpr uint32_t Timer11 = Apb2Timer;

	static bool inline
	enable()
	{
		ClockControl::enableInternalClock();	// 16MHz
		ClockControl::enablePll(
			ClockControl::PllSource::InternalClock,
		    16,	 //  16MHz / M=16  ->   1MHz
		    336, //   1MHz * N=336 -> 336MHz
		    4,	 // 336MHz / P=4   ->  84MHz = F_cpu
		    7	 // 336MHz / Q=7   ->  48MHz = F_usb
		);
		// set flash latency for 84MHz
		ClockControl::setFlashLatency(Frequency);
		// switch system clock to PLL output
		ClockControl::enableSystemClock(ClockControl::SystemClockSource::Pll);
		ClockControl::setAhbPrescaler(ClockControl::AhbPrescaler::Div1);
		// APB1 has max. 42MHz
		ClockControl::setApb1Prescaler(ClockControl::Apb1Prescaler::Div2);
		ClockControl::setApb2Prescaler(ClockControl::Apb2Prescaler::Div1);
		// update frequencies for busy-wait delay functions
		xpcc::clock::fcpu     = Frequency;
		xpcc::clock::fcpu_kHz = Frequency / 1000;
		xpcc::clock::fcpu_MHz = Frequency / 1000000;
		xpcc::clock::ns_per_loop = 36; // 3000 / 84 = 35.714 = ~36ns per delay loop

		return true;
	}
};

// Arduino Footprint
using A0 = GpioA0;
using A1 = GpioA1;
using A2 = GpioA4;
using A3 = GpioB0;
using A4 = GpioC1;
using A5 = GpioC0;

using D0  = GpioA3;
using D1  = GpioA2;
using D2  = GpioA10;
using D3  = GpioB3;
using D4  = GpioB5;
using D5  = GpioB4;
using D6  = GpioB10;
using D7  = GpioA8;
using D8  = GpioA9;
using D9  = GpioC7;
using D10 = GpioB6;
using D11 = GpioA7;
using D12 = GpioA6;
using D13 = GpioA5;
using D14 = GpioB9;
using D15 = GpioB8;

using Button = xpcc::GpioInverted<GpioInputC13>;
using LedD2 = D13;

using Leds = xpcc::SoftwareGpioPort< LedD2 >;


namespace stlink
{
using Rx = GpioInputA3;
using Tx = GpioOutputA2;
using Uart = Usart2;
}


inline void
initialize()
{
	systemClock::enable();
	xpcc::cortex::SysTickTimer::initialize<systemClock>();

	stlink::Tx::connect(stlink::Uart::Tx);
	stlink::Rx::connect(stlink::Uart::Rx, Gpio::InputType::PullUp);
	stlink::Uart::initialize<systemClock, 115200>(12);

	Button::setInput();
	Button::setInputTrigger(Gpio::InputTrigger::RisingEdge);
	Button::enableExternalInterrupt();
//	Button::enableExternalInterruptVector(12);
}

}

#endif	// XPCC_STM32_NUCLEO_F401RE_HPP
