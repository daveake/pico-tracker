#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "types.h"
#include "misc.h"

#define I2C_PORT i2c0

static int DeviceAddress = 0;	// We check on both address 0x76 and 0x77

int32_t t_fine;

uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint8_t dig_H1, dig_H3;
int8_t dig_H6;
int16_t dig_H2, dig_H4, dig_H5;

/* The following compensation functions are required to convert from the raw ADC
data from the chip to something usable. Each chip has a different set of
compensation parameters stored on the chip at point of manufacture, which are
read from the chip at startup and used inthese routines.
*/
int32_t compensate_temp(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T =(t_fine*5+128)>>8;
    return T;
}

uint32_t compensate_pressure(int32_t adc_P) {
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
    var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18; var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
    if (var1 == 0)
        return 0;

    p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t)var1);
    else
        p = (p / (uint32_t)var1) * 2;

    var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12; var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));

    return p;
}

uint32_t compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
                ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *
                ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12);
}

static void write_register(uint8_t reg, uint8_t data)
{
	uint8_t buffer[2];
	
	buffer[0] = reg;
	buffer[1] = data;
	
	i2c_write_blocking(I2C_PORT, DeviceAddress, buffer, 2, false);
}

static void read_registers(uint8_t reg, uint8_t *buffer, uint16_t len)
{
	
    i2c_write_blocking(I2C_PORT, DeviceAddress, &reg, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_PORT, DeviceAddress, buffer, len, false);
}

/* This function reads the manufacturing assigned compensation parameters from the device */
void read_compensation_parameters()
{
    uint8_t buffer[26];

    read_registers(0x88, buffer, 24);

    dig_T1 = buffer[0] | (buffer[1] << 8);
    dig_T2 = buffer[2] | (buffer[3] << 8);
    dig_T3 = buffer[4] | (buffer[5] << 8);

    dig_P1 = buffer[6] | (buffer[7] << 8);
    dig_P2 = buffer[8] | (buffer[9] << 8);
    dig_P3 = buffer[10] | (buffer[11] << 8);
    dig_P4 = buffer[12] | (buffer[13] << 8);
    dig_P5 = buffer[14] | (buffer[15] << 8);
    dig_P6 = buffer[16] | (buffer[17] << 8);
    dig_P7 = buffer[18] | (buffer[19] << 8);
    dig_P8 = buffer[20] | (buffer[21] << 8);
    dig_P9 = buffer[22] | (buffer[23] << 8);

    dig_H1 = buffer[25];

    read_registers(0xE1, buffer, 8);

    dig_H2 = buffer[0] | (buffer[1] << 8);
    dig_H3 = (int8_t)buffer[2];
    dig_H4 = buffer[3] << 4 | (buffer[4] & 0xf);
    dig_H5 = (buffer[5] >> 4) | (buffer[6] << 4);
    dig_H6 = (int8_t)buffer[7];
}

static void bme280_read_raw(int32_t *humidity, int32_t *pressure, int32_t *temperature) {
    uint8_t buffer[8];

    read_registers(0xF7, buffer, 8);
    *pressure = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | (buffer[2] >> 4);
    *temperature = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (buffer[5] >> 4);
    *humidity = (uint32_t)buffer[6] << 8 | buffer[7];
}


void setup_bme(void)
{
	int bytes_transferred;
	uint8_t rxdata;

	// Set up I2C and check for device
	printf("  - Init BME  - ");

    // This example will use I2C0 on GPIO4 (SDA) and GPIO5 (SCL) running at 400kHz.
    i2c_init(I2C_PORT, 400*1000);
    gpio_funcsel(12, GPIO_FUNC_I2C);
    gpio_funcsel(13, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);
	
    if (i2c_read_blocking(i2c0, DeviceAddress = 0x76, &rxdata, 1, false) <= 0)
	{
		if (i2c_read_blocking(i2c0, DeviceAddress = 0x76, &rxdata, 1, false) <= 0)
		{
			printf("FAIL (No Device)\n");
			DeviceAddress = 0;
		}
	}

	if (DeviceAddress > 0)
	{
		// Interrogate the device for its I2C ID number, should be 0x60
		uint8_t id;

		read_registers(0xD0, &id, 1);
		
		if (id == 0x60)
		{
			printf("OK (%02Xh)\n", DeviceAddress);
			
			read_compensation_parameters();

			write_register(0xF2, 0x1); // Humidity oversampling register - going for x1
			write_register(0xF4, 0x27);// Set rest of oversampling modes and run mode to normal
		}
		else
		{
			// Wrong type of device
			printf("FAIL (Wrong Device)\n");
			DeviceAddress = 0;
		}
	}
}

void check_bme(struct TGPS *GPS)
{
	if (DeviceAddress > 0)
	{
		static uint64_t NextAction = 0;

		if (get_time() > NextAction)
		{
			int32_t humidity, pressure, temperature;

			bme280_read_raw(&humidity, &pressure, &temperature);

			// These are the raw numbers from the chip, so we need to run through the
			// compensations to get human understandable numbers
			pressure = compensate_pressure(pressure);
			temperature = compensate_temp(temperature);
			humidity = compensate_humidity(humidity);

			// printf("\nHumidity = %.2f%%\n",humidity / 1024.0);
			// printf("Pressure = %dPa\n", pressure);
			// printf("Temp. = %.2fC\n\n", temperature/100.0);
			
			GPS->ExternalTemperature = temperature / 100.0;
			GPS->Pressure = pressure;
			GPS->Humidity = humidity / 1024.0;

			NextAction = get_time() + 1000000L;
		}
	}
}
