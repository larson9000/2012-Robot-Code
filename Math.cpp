#include <WPILib.h>
#include "Math.h"

double degToRad(double degrees)
{
	return degrees * PI / 180.0;
}

double radToDeg(double radians)
{
	return radians * 180.0 / PI;
}

void rotate(double x, double y, double angle, double* rx, double* ry)
{
	angle = degToRad(angle);
	*rx = x * std::cos(angle) - y * std::sin(angle);
	*ry = x * std::sin(angle) + y * std::cos(angle);
}
