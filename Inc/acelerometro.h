
#include "acelerometro.h"
#include "FreeRtos.h"
#include "semphr.h"

double X = 0;
double Y = 0;
double Z = 0;

SemaphoreHandle_t accelMutex;

void Obtain_Coordinates_XYZ()
/* Calculate the coordinates X Y Z, and sotres them in the global variables X Y Z */
{
  int Ix, Iy, Iz;
  uint8_t Ix1, Ix2;
  uint8_t Iy1, Iy2;
  uint8_t Iz1, Iz2;
	
	Ix1 = SPI_Read (0x28);
	Ix2 = SPI_Read (0x29);
	Ix = (Ix2 << 8) + Ix1;
	if (Ix >= 0x8000) Ix = -(65536 - Ix);
	X = Ix/16384.0;
		
	Iy1 = SPI_Read (0x2A);
	Iy2 = SPI_Read (0x2B);
	Iy = (Iy2 << 8) + Iy1;
	if (Iy >= 0x8000) Iy = -(65536 - Iy);
	Y = Iy/16384.0;
		
	Iz1 = SPI_Read (0x2C);
	Iz2 = SPI_Read (0x2D);
	Iz = (Iz2 << 8) + Iz1;
	if (Iz >= 0x8000) Iz = -(65536 - Iz);
	Z = Iz/16384.0;	
}

void RefreshAcc(double* acc_X, double* acc_Y, double* acc_Z) {
	xSemaphoreGive(accelMutex);
		Obtain_Coordinates_XYZ();	
		*acc_X = X;
		*acc_Y = Y;
		*acc_Z = Z;
	xSemaphoreGive(accelMutex);
}

double Calculate_RotationX ()
{ 
	double rotX;
	xSemaphoreTake(accelMutex, portMAX_DELAY );
	Obtain_Coordinates_XYZ();	
	rotX = atan2(Y, sqrt(X*X+Z*Z)) * 180.0/3.1416;
	xSemaphoreGive(accelMutex);
	return rotX;
 }	

double Calculate_RotationY ()
{
  double rotY;
	xSemaphoreTake(accelMutex, portMAX_DELAY );
	Obtain_Coordinates_XYZ();
	rotY = - atan2(X, sqrt(Y*Y+Z*Z)) * 180.0/3.1416;
	xSemaphoreGive(accelMutex);
	return rotY;
 }
