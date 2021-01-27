#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "types.h"
#include "misc.h"
#include "led.h"

const uint LED_PIN = 25;


void setup_led(void)
{
	// Set up our sole LED
	printf("  - Init LED  - ");
	gpio_funcsel(LED_PIN, GPIO_FUNC_PROC);
	gpio_dir(LED_PIN, GPIO_OUT);
	printf("OK\n");
}

void check_led(struct TGPS *GPS)
{
	static uint64_t NextAction = 0;
	static int blink=0;
	uint64_t DelayMS;

	if (get_time() > NextAction)
	{
		if (GPS->Altitude > 2000)
		{
			// LED off to save a teeny weeny amount of power
			DelayMS = 10000;
			blink = 0;
		}
		else
		{
			DelayMS = (GPS->Satellites >= 4) ? 1000 : 250;
			blink ^= 1;
		}

		gpio_put(LED_PIN, blink);
		NextAction = get_time() + DelayMS * 1000;
	}
}
