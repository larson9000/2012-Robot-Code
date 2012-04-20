#ifndef ROBOT_H
#define ROBOT_H

#include <WPILib.h>
#include "Math.h"
#include "JoystickCallback.h"
#include "LSM303_I2C.h"
#define GET_FUNC(x) &Robot::x

class Vision;

class AccelPID_Wrapper : public PIDSource
{
public:
	AccelPID_Wrapper(ADXL345_I2C *member) { internal = member; }
	virtual ~AccelPID_Wrapper() { delete internal; }
	
	double PIDGet() { return atan2(internal->GetAccelerations().YAxis,internal->GetAccelerations().XAxis)-1.5; } //86 degrees is "down"
	
private:
	ADXL345_I2C *internal;
};

/**
 * Our main robot class.
 */
class Robot : public SimpleRobot
{
public:
	Robot();
	~Robot();
	void Autonomous();
	void OperatorControl();
	
private:
	void BalanceRobotOff();
	void BalanceRobotOn();
	void RampDown();
	void RampUp();
	void CollectorEject();
	void RampOff();
	void ShootBasket(int shots);
	void ShootBasketTeleoperated();
	void MoveTurret();
	void TurretOff();
	void HalfSpeedOn();
	void QuarterSpeedOn();
	void NormalSpeed();
	void RatioDown();
	void RatioUp();
	void CollectorIncBall();
	void CollectorDecBall();

	double shotDirectionModifier();
	double shotDistanceModifier();
	void ShotXInc();
	void ShotXDec();
	void ShotZInc();
	void ShotZDec();

	void reservePrimaryLines();
	void reserveSecondaryLines();
	
	DisplayWriter primaryDisplay;
	DisplayWriter secondaryDisplay;
	
	AccelPID_Wrapper*			balanceAccelerometer;
	Vision*						vision;
	Gyro*						gyro;
	PIDController*				balancePID;
	JoystickCallback<Robot>*	joystickCallbackHandler;
	JoystickWrapper*			joystick1;
	JoystickWrapper*			joystick2;
	double 						speedMultiplier;
	int							shotModifierX;
	int							shotModifierZ;
};

#endif // ROBOT_H
