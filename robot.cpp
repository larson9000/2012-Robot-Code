#include "Collector.h"
#include "DisplayWriter.h"
#include "DisplayWrapper.h"
#include "DriveTrain.h"
#include "JoystickWrapper.h"
#include "Logger.h"
#include "Robot.h"
#include "Singleton.h"
#include "squarefinder.h"
#include <time.h>
#include "SharpIR.h"
#include "Shooter.h"

Robot::Robot()
{
	GetWatchdog().SetEnabled(false); ///\todo enable this before the competition

	speedMultiplier = 1.0;
	shotModifierX = 0;			// start with a count of left/right adjustment in .01 increments
	shotModifierZ = 0;			// start with a count of distance adjustments in 6" increments

	Logger* logger = new Logger("/ni-rt/system/logs/robot.txt");
	Singleton<Logger>::SetInstance(logger);

	vision = new Vision(new SquareFinder);
	vision->start();
	Singleton<DriveTrain>::SetInstance(new DriveTrain);
	Singleton<Collector>::SetInstance(new Collector);
	Singleton<Collector>::GetInstance().Start();
	Singleton<Shooter>::SetInstance(new Shooter);

	// The order in which lines are reserved dictates the order
	// in which lines are displayed on the LCD.
	this->primaryDisplay.reserve(1);
	COLLECTOR.reservePrimaryLines();
	SHOOTER.reservePrimaryLines();
	DRIVETRAIN.reservePrimaryLines();
	VISION.reservePrimaryLines();
	SQUAREFINDER.reservePrimaryLines();

	// There is some additional information, secondary information,
	// that comes after all of the primary information.
	this->secondaryDisplay.reserve(7);
	COLLECTOR.reserveSecondaryLines();
	SHOOTER.reserveSecondaryLines();
	DRIVETRAIN.reserveSecondaryLines();
	VISION.reserveSecondaryLines();
	SQUAREFINDER.reserveSecondaryLines();

	Singleton<Logger>::GetInstance().Logf("Starting the Robot class.");

	//balanceAccelerometer = new AccelPID_Wrapper(new ADXL345_I2C(1)); //Takes ownership of ADXL345

	//balancePID = new PIDController(0.1,.01,0.0,balanceAccelerometer,&Singleton<DriveTrain>::GetInstance()); //\todo Tune these! No D.
	//balancePID->Disable();

	//gyro = new Gyro(1);
	//gyro->Reset();

	joystick1 = new JoystickWrapper(1, Extreme3DPro);
	//joystick2 = new JoystickWrapper(2, Attack3);
	joystickCallbackHandler = new JoystickCallback<Robot>(joystick1,this);
	joystickCallbackHandler->SetHeldCallback(RAMP_DOWN_BUTTON, GET_FUNC(RampDown));
	joystickCallbackHandler->SetHeldCallback(RAMP_UP_BUTTON, GET_FUNC(RampUp));
	joystickCallbackHandler->SetHeldCallback(EJECT_BALLS_BUTTON, GET_FUNC(CollectorEject));
	joystickCallbackHandler->SetUpCallback(RAMP_DOWN_BUTTON, GET_FUNC(RampOff));
	joystickCallbackHandler->SetUpCallback(RAMP_UP_BUTTON, GET_FUNC(RampOff));
	joystickCallbackHandler->SetTriggerDownCallback(GET_FUNC(ShootBasketTeleoperated));
	joystickCallbackHandler->SetHeldCallback(TURRET_BUTTON, GET_FUNC(MoveTurret));
	joystickCallbackHandler->SetUpCallback(TURRET_BUTTON, GET_FUNC(TurretOff));

	joystickCallbackHandler->SetHeldCallback(9, GET_FUNC(RatioDown));
	joystickCallbackHandler->SetHeldCallback(11, GET_FUNC(RatioUp));

	// Change Ball Count
	joystickCallbackHandler->SetDownCallback(COLLECTOR_ADD_BALL_BUTTON, GET_FUNC(CollectorIncBall));
	joystickCallbackHandler->SetDownCallback(COLLECTOR_SUB_BALL_BUTTON, GET_FUNC(CollectorDecBall));

	/*
	 joystickCallbackHandler->SetDownCallback(11, GET_FUNC(HalfSpeedOn));
	 joystickCallbackHandler->SetDownCallback(12, GET_FUNC(QuarterSpeedOn));
	 joystickCallbackHandler->SetUpCallback(11, GET_FUNC(NormalSpeed));
	 joystickCallbackHandler->SetUpCallback(12, GET_FUNC(NormalSpeed));
	 */
	//joystickCallbackHandler->SetDownCallback(BalanceRobot,GET_FUNC(BalanceRobotOn));
	//joystickCallbackHandler->SetUpCallback(BalanceRobot,GET_FUNC(BalanceRobotOff));
}
//Robot used to be 5200B processor.
Robot::~Robot()
{
	Singleton<Logger>::GetInstance().Logf("Shutting down the Robot class.");

	//Destroy instances of singletons that we have used
	Singleton<Collector>::DestroyInstance();
	Singleton<DisplayWrapper>::DestroyInstance();
	Singleton<DriveTrain>::DestroyInstance();
	Singleton<Logger>::DestroyInstance();
	Singleton<Shooter>::DestroyInstance();

	delete joystickCallbackHandler;

	//delete balancePID;
	//delete balanceAccelerometer;
}

void Robot::RampDown()
{
	ROBOT.primaryDisplay.printfLine(0, "Ramp Going Down");
	Singleton<Collector>::GetInstance().ManipulateRamp(DOWN);
}

void Robot::RampUp()
{
	ROBOT.primaryDisplay.printfLine(0, "Ramp Going Up");
	Singleton<Collector>::GetInstance().ManipulateRamp(UP);
}

void Robot::CollectorIncBall()
{
	Singleton<Collector>::GetInstance().ChangeBallCountBy(1);
}

void Robot::CollectorDecBall()
{
	Singleton<Collector>::GetInstance().ChangeBallCountBy(-1);
}

double Robot::shotDirectionModifier()
{
	// offset in hundredths left or right of target
	return shotModifierX * .01;
}

double Robot::shotDistanceModifier()
{
	// offset in 6" distance increments
	return shotModifierZ * 0.5;
}

void Robot::ShotXInc()
{
	shotModifierX += 1;
}

void Robot::ShotXDec()
{
	shotModifierX -= 1;
}

void Robot::ShotZInc()
{
	shotModifierZ += 1;
}

void Robot::ShotZDec()
{
	shotModifierZ -= 1;
}


void Robot::RampOff()
{
	ROBOT.primaryDisplay.printfLine(0, "Ramp Turning Off");
	Singleton<Collector>::GetInstance().ManipulateRamp(RAMP_OFF);
}

void Robot::CollectorEject()
{
	ROBOT.primaryDisplay.printfLine(0, "Ejecting Ball");
	Singleton<Collector>::GetInstance().Eject();
}

void Robot::Autonomous()
{
	Singleton<Logger>::GetInstance().Logf("Starting Autonomous Mode.");
	Singleton<Collector>::GetInstance().SetBallCount( 2); // preloaded with 2 balls in autonomous

	primaryDisplay.printfLine(0, "Shooting 2");
	ShootBasket( 2 );

	KinectStick leftStick(1);
	KinectStick rightStick(2);

	Timer displayUpdateFrequency;
	displayUpdateFrequency.Start();

	primaryDisplay.printfLine(0, "Kinect Controls");
	while (IsAutonomous() )
	{
		secondaryDisplay.printfLine(1, "Shot-Dir: %.2f", shotDirectionModifier());
		secondaryDisplay.printfLine(2, "Shot-Dist: %.1f\"", shotDistanceModifier());

		DRIVETRAIN.SetLeft(-leftStick.GetY());
		DRIVETRAIN.SetRight(rightStick.GetY());
		if (leftStick.GetRawButton(1) ) // Left Leg Out
		{
			primaryDisplay.printfLine(0, "Ramp UP");
			RampUp();
		}

		if (leftStick.GetRawButton(2) ) // Right Leg Out
		{
			primaryDisplay.printfLine(0, "Ramp DOWN");
			RampDown();
		}

		if ( !leftStick.GetRawButton(1) && !leftStick.GetRawButton(2) ) // both legs in
			RampOff();

		// The Joystick Throttle controls the scrolling of the display
		// The display is updated at a controlled pace
		DisplayWrapper::GetInstance()->setScrollLocation(joystick1->GetThrottle());
		if (displayUpdateFrequency.HasPeriodPassed(1.0 / 5)) {
			secondaryDisplay.printfLine(1, "Shot-Dir: %.2f", shotDirectionModifier());
			secondaryDisplay.printfLine(2, "Shot-Dist: %.1f\'", shotDistanceModifier());

			displayUpdateFrequency.Reset();
			DisplayWrapper::GetInstance()->output();
		}

		Wait(0.1);
	}

	Singleton<Logger>::GetInstance().Logf("Stopping Autonomous Mode.");
}

void Robot::OperatorControl()
{
	LOGGER.Logf("Starting operator control.");
	DriveTrain& driveTrain = Singleton<DriveTrain>::GetInstance();

	Timer displayUpdateFrequency;
	displayUpdateFrequency.Start();

	ROBOT.primaryDisplay.printfLine(0, "Joystick Controls");
	while (IsOperatorControl())
	{
		joystickCallbackHandler->Update();
		float x, y, rot;
		joystick1->GetAxis(&x, &y);
		rot = joystick1->GetRawRotation();

		if ( !joystick1->GetButton(TURRET_BUTTON) )
			driveTrain.DriveArcade(speedMultiplier*rot, speedMultiplier
					*cubicFilter(-y));

		secondaryDisplay.printfLine(2, "X/Y/R: %f/%f/%f", x, y, rot);
		secondaryDisplay.printfLine(3, "rps:%f", ((joystick1->GetThrottle() + 1.0) / 2.0) * 15.0 + 20.0);
		secondaryDisplay.printfLine(4, "ratio:%f", Singleton<Shooter>::GetInstance().GetTopRatio());

		double offset, distance;
		Singleton<Vision>::GetInstance().FindTarget(offset, distance);
		secondaryDisplay.printfLine(5, "Speed:%f", SIGN(offset)*.15+.1*offset);
		secondaryDisplay.printfLine(6, "Vis:%1.4f,%1.4", offset, distance);

		Singleton<Shooter>::GetInstance().Update();

		// The display is updated at a controlled pace
		if (displayUpdateFrequency.HasPeriodPassed(1.0 / 5)) {
			// this is a convenient time to check the HAT for shot adjustments
			float xaxis, yaxis;
			joystick1->GetPov(&xaxis, &yaxis);
			if (xaxis < 0)
				ShotXDec();
			if (xaxis > 0)
				ShotXInc();
			if (yaxis < 0)
				ShotZDec();
			if (yaxis > 0)
				ShotZInc();

			displayUpdateFrequency.Reset();

			secondaryDisplay.printfLine(1, "Shot-Dir: %.2f", shotDirectionModifier());
			secondaryDisplay.printfLine(2, "Shot-Dist: %.1f\'", shotDistanceModifier());

			// The Joystick Throttle controls the scrolling of the display
			DisplayWrapper::GetInstance()->setScrollLocation(joystick1->GetThrottle());

			// update the LCD
			DisplayWrapper::GetInstance()->output();
		}

		Wait(0.01);
	}
	LOGGER.Logf("Stopping operator control.");
}

void Robot::BalanceRobotOff()
{
	balancePID->Disable();
}

void Robot::BalanceRobotOn()
{
	balancePID->Enable();
	balancePID->SetSetpoint(0.0);
}
void Robot::ShootBasketTeleoperated()
{
	ShootBasket( 0 );
}

void Robot::ShootBasket( int shots )
{
	double offset = 0.0;
	double distance = 0.0;
	Shooter& shooter = Singleton<Shooter>::GetInstance();
	Collector& collector = Singleton<Collector>::GetInstance();
	Vision& vision = Singleton<Vision>::GetInstance();

	// disable the drive
	DRIVETRAIN.DriveArcade(0.0, 0.0);

	collector.PrepareToShoot();

	Timer alignTimer;
	alignTimer.Start();
	while(!alignTimer.HasPeriodPassed(3.5) && (joystick1->GetJoystick()->GetRawButton(1) || shots > 0) )
	{
		vision.FindTarget(offset, distance);
		offset += shotDirectionModifier();
		distance += shotDistanceModifier();

		ROBOT.secondaryDisplay.printfLine(5, "Speed:%f", SIGN(offset)*.15+.1*offset);
		ROBOT.secondaryDisplay.printfLine(6, "Vis:%1.4f,%1.4", offset, distance);
		DisplayWrapper::GetInstance()->output();
		if (fabs(offset) < 0.02 && distance != 0)
			break;
		shooter.SetTurret(-1.0*(SIGN(offset)*/*.16*/.175));
		Wait(0.05);
		shooter.SetTurret(0.0);
		if( fabs(offset) <= .2)
			Wait(.2 - fabs(offset));
	}
	shooter.SetTurret(0.0);
    // if timer expired or the user has released the trigger button
	if (alignTimer.HasPeriodPassed(3.5) || ( !joystick1->GetJoystick()->GetRawButton(1) && shots == 0 ))
		return;

	shooter.ShootBasket(distance, joystick1->GetJoystick() , shots );
	//shooter.Shoot(27.7);
}

void Robot::MoveTurret()
{
	float rot = joystick1->GetRawRotation() / -2.0;
	Singleton<Shooter>::GetInstance().SetTurret(rot);
	Singleton<DriveTrain>::GetInstance().DriveArcade(0.0, 0.0);
}

void Robot::TurretOff()
{
	SHOOTER.SetTurret(0);
}

void Robot::HalfSpeedOn()
{
	speedMultiplier = 0.5;
}

void Robot::QuarterSpeedOn()
{
	speedMultiplier = 0.25;
}

void Robot::NormalSpeed()
{
	speedMultiplier = 1.0;
}

void Robot::RatioDown()
{
	SHOOTER.SetTopRatio(SHOOTER.GetTopRatio() * 0.99);
}

void Robot::RatioUp()
{
	SHOOTER.SetTopRatio(SHOOTER.GetTopRatio() * 1.01);
}

START_ROBOT_CLASS(Robot)
