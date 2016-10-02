/* Copyright (c) {{ year }}, {{ your name }}
 * All Rights Reserved.
 *
 * The file is part of the xpcc library and is released under the 3-clause BSD
 * license. See the file `LICENSE` for the full license governing this code.
 */
#include <xpcc/architecture/platform.hpp>

// Leave this function empty for now
extern "C" void xpcc_gpio_enable(void) {}

int main()
{
    // Initialize one GPIO output manually using the datasheet.
    // Don't forget to enable the GPIO clock!
    // GPIO_PORTA_OUTPUT = (1 << 0);

    while(1)
    {
        // Toggle the pin manually
        // GPIO_PORTA_TOGGLE = (1 << 0);

        // Create a very crude busy wait. The numbers don't have to
        // be super accurate, just in the ball-park of a 1s delay.
        // Use the initial CPU frequency in MHz here.
        constexpr uint32_t fcpu_MHz = 8;
        for (volatile uint32_t ii = 0; ii < fcpu_MHz * 250000; ii++) ;
    }
}
