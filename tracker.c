#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "types.h"
#include "misc.h"
#include "battery.h"
#include "led.h"
#include "bme280.h"
#include "gps.h"
#include "lora.h"
#include "prediction.h"

// Tracker configuration


// Variables
struct TGPS GPS;

int ProcessFieldCommand(char *Line)
{
	int OK = 0;

	if (Line[0] == 'P')
	{
		GPS.PreviousAltitude = GPS.Altitude;
		sscanf(Line+1,"%f,%f,%ld", &GPS.Latitude, &GPS.Longitude, &GPS.Altitude);
		GPS.UseHostPosition = 5;
		GPS.AscentRate = GPS.AscentRate * 0.7 + (GPS.Altitude - GPS.PreviousAltitude) * 0.3;
		OK = 1;
	}

	return OK;
}

void ProcessCommand(char *Line)
{
	int OK = 0;

	if (Line[0] == 'G')
	{
		// OK = ProcessGPSCommand(Line+1);
	}
	else if (Line[0] == 'C')
	{
		// OK = ProcessCommonCommand(Line+1);
	}
	else if (Line[0] == 'L')
	{
		// OK = ProcessLORACommand(Line+1);
	}
	else if (Line[0] == 'A')
	{
		// OK = ProcessAPRSCommand(Line+1);
	}
	else if (Line[0] == 'F')
	{
		OK = ProcessFieldCommand(Line+1);
	}

	if (OK)
	{
		printf("*\n");
	}
	else
	{
		printf("?\n");
	}
}

void check_host(void)
{
	static char Line[80];
	static unsigned int Length=0;
	char Character;

	while (uart_readable(uart0))
	{
		char Character;
		
		Character = uart_getc(uart0);
		
		if (Character == '~')
		{
		  Line[0] = Character;
		  Length = 1;
		}
		else if (Character == '\r')
		{
		  Line[Length] = '\0';
		  ProcessCommand(Line+1);
		  Length = 0;
		}
		else if (Length >= sizeof(Line))
		{
		  Length = 0;
		}
		else if (Length > 0)
		{
		  Line[Length++] = Character;
		}
	}
}

int main()
{
	
	setup_default_uart();
	printf("\nPi Pico HAB Tracker V1.0");
	printf("\n========================\n\n");

	// Init modules
	printf("Initialisation ...\n\n");
	setup_adc();
	setup_led();
	setup_bme();
	setup_gps();
	setup_lora(434.450, 1, "SANTA");
	setup_prediction(&GPS);
	
	printf("\nTracker Running ...\n\n");
	
	while (1)
	{			
		check_adc(&GPS);
		check_led(&GPS);
		check_bme(&GPS);
		check_gps(&GPS);
		check_lora(&GPS);
		check_host();
		check_prediction(&GPS);
	}
}
