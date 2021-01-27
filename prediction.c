#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "types.h"
#include "misc.h"
#include "prediction.h"

#define DEG2RAD (3.142 / 180)
#define SLOTSIZE 100
#define SLOTS 45000 / SLOTSIZE
#define POLL_PERIOD 5

#define INITIAL_CDA         0.7
#define PAYLOAD_WEIGHT      1.0
#define LANDING_ALTITUDE    100

struct TPosition
{
	float LatitudeDelta;
	float LongitudeDelta;
};

struct TPosition Positions[SLOTS];		// 100m slots from 0 to 45km

float PreviousLatitude, PreviousLongitude, cd_area;
unsigned long PreviousAltitude;

int GetSlot(int32_t Altitude)
{
  int Slot;
  
  Slot = Altitude / SLOTSIZE;
  
  if (Slot < 0) Slot = 0;
  if (Slot >= SLOTS) Slot = SLOTS-1;
  
  return Slot;
}

float CalculateAirDensity(float Altitude)
{
	float Temperature, Pressure;

  if (Altitude < 11000.0)
  {
      // below 11Km - Troposphere
      Temperature = 15.04 - (0.00649 * Altitude);
      Pressure = 101.29 * pow((Temperature + 273.1) / 288.08, 5.256);
  }
  else if (Altitude < 25000.0)
  {
    // between 11Km and 25Km - lower Stratosphere
    Temperature = -56.46;
    Pressure = 22.65 * exp(1.73 - ( 0.000157 * Altitude));
  }
  else
  {
    // above 25Km - upper Stratosphere
    Temperature = -131.21 + (0.00299 * Altitude);
    Pressure = 2.488 * pow((Temperature + 273.1) / 216.6, -11.388);
  }
  
  return Pressure / (0.2869 * (Temperature + 273.1));
}

float CalculateDescentRate(float Weight, float CDTimesArea, float Altitude)
{
  float Density;
  
  Density = CalculateAirDensity(Altitude);
  
  return sqrt((Weight * 9.81)/(0.5 * Density * CDTimesArea));
}

float CalculateCDA(float Weight, float Altitude, float DescentRate)
{
  float Density;
  
  Density = CalculateAirDensity(Altitude);
  
  printf("Alt %f, Rate %f, CDA %f\n", Altitude, DescentRate, (Weight * 9.81)/(0.5 * Density * DescentRate * DescentRate));
  
  return (Weight * 9.81)/(0.5 * Density * DescentRate * DescentRate);
}

int CalculateLandingPosition(float CDA, float Latitude, float Longitude, int32_t Altitude, float *PredictedLatitude, float *PredictedLongitude)
{
	float TimeTillLanding, TimeInSlot, DescentRate;
	int Slot;
	int32_t DistanceInSlot;

	TimeTillLanding = 0;

	Slot = GetSlot(Altitude);
	DistanceInSlot = Altitude + 1 - (Slot * SLOTSIZE);

	while (Altitude > LANDING_ALTITUDE)
	{
		Slot = GetSlot(Altitude);

		if (Slot == GetSlot(LANDING_ALTITUDE))
		{
			DistanceInSlot = Altitude - LANDING_ALTITUDE;
		}

		DescentRate = CalculateDescentRate(PAYLOAD_WEIGHT, CDA, Altitude);
		TimeInSlot = DistanceInSlot / DescentRate;

		Latitude += Positions[Slot].LatitudeDelta * TimeInSlot;
		Longitude += Positions[Slot].LongitudeDelta * TimeInSlot;

		// printf("SLOT %d: alt %lu, lat=%f, long=%f, rate=%f, dist=%lu, time=%f\n", Slot, Altitude, Latitude, Longitude, DescentRate, DistanceInSlot, TimeInSlot);

		TimeTillLanding = TimeTillLanding + TimeInSlot;
		Altitude -= DistanceInSlot;
		DistanceInSlot = SLOTSIZE;
	}
		
	*PredictedLatitude = Latitude;
	*PredictedLongitude = Longitude;

	return TimeTillLanding; 
}

void setup_prediction(struct TGPS *GPS)
{
  PreviousLatitude = 0;
  PreviousLongitude = 0;
  PreviousAltitude = 0;
  
  GPS->CDA = INITIAL_CDA;
}

void check_prediction(struct TGPS *GPS)
{
	static uint64_t NextAction = 0;

  
	if ((get_time() > NextAction) && (GPS->Satellites >= 4) && (GPS->Latitude >= -90) && (GPS->Latitude <= 90) && (GPS->Longitude >= -180) && (GPS->Longitude <= 180))
	{
		int Slot;

		NextAction = get_time() + POLL_PERIOD * 1000000L;

		printf("FLIGHT MODE = %d ***\n", GPS->FlightMode);

		if ((GPS->FlightMode >= fmLaunched) && (GPS->FlightMode < fmLanded))
		{
			// Ascent or descent?
			if (GPS->FlightMode == fmLaunched)
			{
				// Going up - store deltas
				Slot = GetSlot(GPS->Altitude/2 + PreviousAltitude/2);

				// Deltas are scaled to be horizontal distance per second (i.e. speed)
				Positions[Slot].LatitudeDelta = (GPS->Latitude - PreviousLatitude) / POLL_PERIOD;
				Positions[Slot].LongitudeDelta = (GPS->Longitude - PreviousLongitude) / POLL_PERIOD;
				printf("Slot %d (%ld): %f, %f\n", Slot, GPS->Altitude, Positions[Slot].LatitudeDelta, Positions[Slot].LongitudeDelta);
			}
			else if ((GPS->FlightMode >= fmDescending) && (GPS->FlightMode <= fmLanding))
			{
				// Coming down - try and calculate how well chute is doing

				GPS->CDA = (GPS->CDA*4 + CalculateCDA(PAYLOAD_WEIGHT, GPS->Altitude/2 + PreviousAltitude/2, (PreviousAltitude - GPS->Altitude) / POLL_PERIOD)) / 5;
			}

			// Estimate landing position
			GPS->TimeTillLanding = CalculateLandingPosition(GPS->CDA, GPS->Latitude, GPS->Longitude, GPS->Altitude, &(GPS->PredictedLatitude), &(GPS->PredictedLongitude));

			GPS->PredictedLandingSpeed = CalculateDescentRate(PAYLOAD_WEIGHT, GPS->CDA, LANDING_ALTITUDE);

			printf("Expected Descent Rate = %4.1f (now) %3.1f (landing), time till landing %d\n", 
						CalculateDescentRate(PAYLOAD_WEIGHT, GPS->CDA, GPS->Altitude),
						GPS->PredictedLandingSpeed, GPS->TimeTillLanding);

			printf("Current    %f, %f, alt %ld\n", GPS->Latitude, GPS->Longitude, GPS->Altitude);
			printf("Prediction %f, %f, CDA %f\n", GPS->PredictedLatitude, GPS->PredictedLongitude, GPS->CDA);
		}
		  
		PreviousLatitude = GPS->Latitude;
		PreviousLongitude = GPS->Longitude;
		PreviousAltitude = GPS->Altitude;
	}
}  

