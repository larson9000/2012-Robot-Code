#include "Constants.h"
#include "Logger.h"
#include "Math.h"
#include "Shooter.h"
#include "Singleton.h"

using std::atan;
using std::cos;
using std::pow;
using std::sin;
using std::sqrt;
using std::tan;

/**
 * Find the perfect speed to shoot the ball into a basket.
 * 
 * \note all units are in FEET.
 * \param distance the distance to the target.
 * \param elevation the height of the target relative to the shooter.
 * \param shootAngle the angle that the ball is being shot at.
 * \param basketRadius the radius of the basket.
 * \param ballRadius the radius of the ball.
 * \return the velocity that the ball should be shot at (in ft/s).
 */
static double GetShootVelocity(double distance, double elevation, double shootAngle, double basketRadius, double ballRadius)
{
	static double rootG = sqrt(32.0);
	static double sinAngle = sin(2 * shootAngle);
	static double cosAngle = cos(2 * shootAngle);
	static double tanAngle = tan(shootAngle);
	//double impactAngle = atan( tan(shootAngle) - (2 * elevation) / distance);
	double impactAngle = atan( tanAngle - 2 * elevation / distance );
	double variance = basketRadius - (ballRadius / sin(impactAngle));
	//double velocity = sqrt( (g * pow(distance + variance, 2.0)) / (2.0 * (distance + variance) * sin(shootAngle) * cos(shootAngle) - elevation * pow(cos(shootAngle) , 2.0) ) );
	return rootG * fabs(distance + variance)/ sqrt(fabs(cosAngle*elevation+elevation-sinAngle*distance-variance*sinAngle));
}

Shooter::Shooter()
{
	Singleton<Logger>::GetInstance().Logf("Shooter: Starting up...");
	//Setup Jaguars
	this->topJag = new Jaguar(DIGITAL_SIDECAR_SLOT, SHOOTER_TOP_JAG_CHANNEL);
	this->bottomJag = new Jaguar(DIGITAL_SIDECAR_SLOT, SHOOTER_BOTTOM_JAG_CHANNEL);
}

Shooter::~Shooter()
{
	Singleton<Logger>::GetInstance().Logf("Shooter: Shutting down...");
	delete this->topJag;
	delete this->bottomJag;
}

void Shooter::Shoot(double speed)
{
	Singleton<Logger>::GetInstance().Logf("Shooting a ball with speed [%f ft/s] and spin [%f Hz]", speed, 0.0);
	///\todo Check inversion.
	this->topJag->Set(-1.0 * speed / MAX_SHOOTER_SPEED);
	this->bottomJag->Set(speed / MAX_SHOOTER_SPEED);
}

void Shooter::ShootBasket(double distance, int level)
{
	Singleton<Logger>::GetInstance().Logf("Shooting a basket at distance [%f ft] at level [%d]", distance, level);
	
	//Calculate the ideal velocity
	double basketElevation;
	switch( level )
	{
		case 1:
			basketElevation = BASKET_1_ELEVATION;
			break;
			
		case 2:
			basketElevation = BASKET_2_ELEVATION;
			break;
			
		case 3:
			basketElevation = BASKET_3_ELEVATION;
			break;
			
		default:
			return;
	}
	double elevation = SHOOTER_ELEVATION - basketElevation;
	double velocity = GetShootVelocity(distance, elevation, SHOOTER_ANGLE, BASKET_RADIUS, BALL_RADIUS);
	
	Shoot(velocity);
}
