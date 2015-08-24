#ifndef VECTOR_H
#define VECTOR_H

#include "Arduino.h"

class Vector
{
public:
	Vector();
	Vector(float X, float Y, float Z);
	float x, y, z;

};

#endif