#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "types.h"
#include "misc.h"

void setup_adc(void)
{
	// Set up ADC for battery and temperature sense
	printf("  - Init ADC  - ");

    adc_init();

    adc_gpio_init(29);				// Set battery sense high impedance
	
	adc_enable_temp_sensor(1);		// Enable temperature sense
	
	printf("OK\n");
}

void check_adc(struct TGPS *GPS)
{
	static uint64_t NextAction = 0;

	if (get_time() > NextAction)
	{
		float conversion_factor, voltage, Temperature, Battery;
		uint16_t raw;
		

		// Select ADC input 4 for temperature
		adc_input_select(4);
		raw = adc_read();
		conversion_factor = 3.3f / (1 << 12);
		voltage = raw * conversion_factor;
		
		Temperature = 27 - (voltage - 0.706) / 0.001721;
		
		if ((GPS->InternalTemperature == 0) || (Temperature == 0))
		{
			GPS->InternalTemperature = Temperature;
		}
		else
		{
			GPS->InternalTemperature = GPS->InternalTemperature * 0.9 + Temperature * 0.1;
		}
		
		// printf("\nINTERNAL = %.1f\n", GPS->InternalTemperature);

		// Select ADC input 3 (GPIO29) for battery
		adc_input_select(3);

        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V, and potential divider is 3:1
		
        raw = adc_read();
        conversion_factor = 3.3f * 3.0 / (1 << 12);			// Convert to V
		Battery = raw * conversion_factor;
		
		if ((GPS->BatteryVoltage == 0) || (Battery == 0))
		{
			GPS->BatteryVoltage = Battery;
		}
		else
		{
			GPS->BatteryVoltage = GPS->BatteryVoltage * 0.9 + Battery * 0.1;
		}

        // printf("\nRaw value: 0x%03x, voltage: %.1f V\n", raw, GPS->BatteryVoltage);

		NextAction = get_time() + 1000000L;
	}
}
