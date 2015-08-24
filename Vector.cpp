#include "Vector.h"
#include "Arduino.h"

Vector::Vector(float X, float Y, float Z)
{
	x = X;
	y = Y;
	z = Z;
}

Vector::Vector()
{
	x=0;
	y=0;
	z=0;
}

