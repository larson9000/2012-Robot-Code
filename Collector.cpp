#include <WPILib.h>
#include "Collector.h"
#include "DisplayWriter.h"
#include "Singleton.h"
#include <cstdio>

unsigned Collector::balls = 0;
Victor *Collector::grabber = NULL;
Victor *Collector::lifter = NULL;
SharpIR *Collector::frontIR = NULL;
SharpIR *Collector::frontMiddleIR = NULL;
SharpIR *Collector::middleIR = NULL;
SharpIR *Collector::topIR = NULL;
CollectorState Collector::collectorState = OFF;
Timer*	Collector::prepTimer = NULL;

void Collector::reservePrimaryLines() { primaryDisplay.reserve(1); }
void Collector::reserveSecondaryLines() { secondaryDisplay.reserve(7); }


Collector::Collector()
{
	grabber = new Victor(COLLECTOR_GRABBER_CHANNEL);
	lifter = new Victor(COLLECTOR_LIFTER_CHANNEL);
	frontIR = new SharpIR(1, IR_FRONT_CHANNEL , COLLECTOR_FRONT_SIGNAL_VOLTAGE, COLLECTOR_FRONT_SIGNAL_TOGGLE_COUNT );
	frontMiddleIR = new SharpIR(1, IR_FRONT_MIDDLE_CHANNEL , COLLECTOR_FRONT_MIDDLE_SIGNAL_VOLTAGE, COLLECTOR_FRONT_MIDDLE_SIGNAL_TOGGLE_COUNT );
	middleIR = new SharpIR(1, IR_MIDDLE_CHANNEL, COLLECTOR_MIDDLE_SIGNAL_VOLTAGE, COLLECTOR_MIDDLE_SIGNAL_TOGGLE_COUNT );
	topIR = new SharpIR(1, IR_TOP_CHANNEL , COLLECTOR_TOP_SIGNAL_VOLTAGE, COLLECTOR_TOP_SIGNAL_TOGGLE_COUNT );
	collectorState = OFF;
	prepTimer = new Timer();
	
	//strike1 = new Relay(RAMP_LEFT_SPIKE_RELAY);
	strike2 = new Relay(RAMP_RIGHT_SPIKE_RELAY);
	
	collectorTask = new Task("2502Cl",(FUNCPTR)ThreadLoop);
}

Collector::~Collector()
{
	collectorTask->Stop();
	delete collectorTask;

	delete grabber;
	delete lifter;
	delete frontIR;
	delete frontMiddleIR;
	delete middleIR;
	delete topIR;
	//delete strike1;
	delete strike2;
	delete prepTimer;
}

bool Collector::Shoot()
{
	// First wait until we're ready to shoot.
	for( int count = 0; count < 100 ; count ++ )
	{
		if( collectorState == OFF )
			break;
		Wait( .01 );
	}
	if( collectorState == LOOKING_FOR_BALLS || collectorState == OFF )
	{
		collectorState = SHOOTING;
		// Now wait until we've taken the shot
		for( int count = 0; count < 500 ; count ++ )
		{
			if( collectorState == LOOKING_FOR_BALLS )
				break;
			Wait( .01 );
		}
	}
	else
	{
		return false;
	}
	return true;
}

void Collector::PrepareToShoot()
{
	collectorState = PREPARE_TO_SHOOT;
	prepTimer->Start();
}

void Collector::ManipulateRamp(RampState state)
{
	static int counter = 0;
	counter++;
	if (collectorState == OFF || collectorState == LOOKING_FOR_BALLS)
	{
		grabber->Set(COLLECTOR_STOP);
		lifter->Set(COLLECTOR_STOP);
		/*
		if ( counter % 3 == 0 )
		{
		*/
			if (state == UP)
			{
				//strike1->Set(Relay::kForward);
				strike2->Set(Relay::kReverse);
			}
			else if (state == DOWN)
			{
				//strike1->Set(Relay::kReverse);
				strike2->Set(Relay::kForward);
			}
		//}
		else
		{
			//strike1->Set(Relay::kOff);
			strike2->Set(Relay::kOff);
			collectorState = LOOKING_FOR_BALLS;
		}
	}
	else
		strike2->Set(Relay::kOff);
}


void Collector::SetBallCount( int ballCount )
{
	balls = ballCount;
}

// Turn off everything
void Collector::Stop()
{
	grabber->Set(COLLECTOR_STOP);
	lifter->Set(COLLECTOR_STOP);
	collectorState = OFF;
}

// Ejects ALL Balls - as long as it is running
void Collector::Eject()
{
	collectorState = EJECTING;
}

// If collector is not doing something else, turn it on.
void Collector::Start()
{
	if( collectorState == OFF )
	{
		collectorTask->Start();
		Wait(1);
		collectorState = LOOKING_FOR_BALLS;
	}
}

void Collector::RejectBall()
{
	grabber->Set( COLLECTOR_RUNFAST_REVERSE );
	Wait( .5 );
	grabber->Set( COLLECTOR_STOP );
}

// Thread that runs continuously
void Collector::ThreadLoop()
{
	Timer stage1Timer;
	int numStage2 = 0;
	int numStage1 = 0;
	int numLooking = 0;
	int lastStage = 0;
	while( true )
	{
		COLLECTOR.primaryDisplay.printfLine(0, "Balls:%d", balls);

		COLLECTOR.secondaryDisplay.printfLine(0, "TopIR:%f", topIR->GetVoltage());
		COLLECTOR.secondaryDisplay.printfLine(1, "Stage:%d", (int)collectorState);
		COLLECTOR.secondaryDisplay.printfLine(2, "FrontIR:%f", frontIR->GetVoltage());
		COLLECTOR.secondaryDisplay.printfLine(3, "FrMiddleIR:%f", frontMiddleIR->GetVoltage());
		COLLECTOR.secondaryDisplay.printfLine(4, "MiddleIR:%f", middleIR->GetVoltage());
		
		switch( collectorState )
		{
		case OFF:
			// Reject any balls that show up.
			if( frontIR->Get() == BALL_VISIBLE || frontMiddleIR->Get() == BALL_VISIBLE)
				RejectBall();
			Wait( .01 );
			break;
		case LOOKING_FOR_BALLS:
			grabber->Set(COLLECTOR_STOP);
			lifter->Set(COLLECTOR_STOP);
			// if we mount a sensor up front, we could turn on when we sense a ball if we have less than 2 balls
			if( frontIR->Get() == BALL_VISIBLE || frontMiddleIR->Get() == BALL_VISIBLE)
			{
				if( balls < MAX_BALLS )
				{
					COLLECTOR.secondaryDisplay.printfLine(5, "SWITCH:%f", frontIR->GetVoltage());
					stage1Timer.Reset();
					stage1Timer.Start();
					collectorState = STAGE1;
					numLooking++;
				}
				else
				{
					// already enough balls, kick it out of our way
					RejectBall();
				}
			}
			Wait( .01 );
			break;
		case STAGE1:
			if( balls < MAX_BALLS )
			{
				if( stage1Timer.HasPeriodPassed(4.0) )
				{
					collectorState = LOOKING_FOR_BALLS;
					stage1Timer.Stop();
					stage1Timer.Reset();
					break;
				}
				grabber->Set(COLLECTOR_RUNFAST);
				if( middleIR->Get() == BALL_VISIBLE ) // Does bottom IR sense Ball?
				{
					// probably want to run this one slowly because we want to stop as soon as the IR sensor no longer senses the ball
					grabber->Set(COLLECTOR_RUNSLOW);
					lifter->Set(COLLECTOR_RUNSLOW);
					numStage1++;
					collectorState = STAGE2;
				}
			}
			else
			{
				// Already have enough balls, kick out any balls in the way and go back to looking which will eject any new balls that appear
				collectorState = LOOKING_FOR_BALLS;
				RejectBall();
				stage1Timer.Stop();
				stage1Timer.Reset();
			}
			break;
		case STAGE2:
			// middle is moving now 
			if( balls < MAX_BALLS )//&& topIR->Get() == BALL_NOT_VISIBLE )
			{
				// the middle sensor no longer senses a ball? time to stop and wait for another ball to show up
				if( middleIR->Get() == BALL_NOT_VISIBLE )
				{
					numStage2++;
					Wait(.1);
					grabber->Set(COLLECTOR_STOP);
					lifter->Set(COLLECTOR_STOP);
					collectorState = LOOKING_FOR_BALLS;
					balls++;
				}
			}
			else
			{
				// Already have enough balls, kick out any balls in the way and go back to looking which will eject any new balls that appear
				collectorState = LOOKING_FOR_BALLS;
				RejectBall();
			}

			break;
		case PREPARE_TO_SHOOT:
			// move backwards until top sensor does not sense a ball
			if( middleIR->Get() == BALL_VISIBLE || prepTimer->HasPeriodPassed(0.2) )
			{
				collectorState = LOOKING_FOR_BALLS;
				lifter->Set( COLLECTOR_STOP );
				prepTimer->Stop();
				prepTimer->Reset();
				break;
			}
			lifter->Set( COLLECTOR_RUNSLOW_REVERSE );
			break;

		case SHOOTING:
			lifter->Set( COLLECTOR_RUNFAST );
			
			//This ensures that only one ball can be shot at once
			for( unsigned i = 0; i < 3000; ++i )
			{
				Wait(0.001);
				if( topIR->Get() == BALL_VISIBLE )
					break;
			}
			for( unsigned i = 0; i < 3000; ++i )
			{
				Wait(0.001);
				if( topIR->Get() == BALL_NOT_VISIBLE )
					break;
			}
			
			Wait(0.1);
			
			grabber->Set(COLLECTOR_STOP);
			lifter->Set(COLLECTOR_STOP);
			if (balls >= 1) {
				balls--;
			}
			collectorState = LOOKING_FOR_BALLS;
			break;
			
		case EJECTING:
			grabber->Set(COLLECTOR_RUNFAST_REVERSE);
			lifter->Set(COLLECTOR_RUNFAST_REVERSE);
			Wait( 3.5 );
			grabber->Set(COLLECTOR_STOP);
			lifter->Set(COLLECTOR_STOP);
			balls = 0;
			collectorState = LOOKING_FOR_BALLS;
			break;
		}

		if (lastStage != collectorState) {
			COLLECTOR.secondaryDisplay.printfLine(6, "Collector: %d to %d\n", lastStage, collectorState);
			lastStage = collectorState;
		}
	}	
}

void Collector::ChangeBallCountBy(int c)
{
	int tmpBalls = balls + c;
	if (tmpBalls < 0) {
		tmpBalls = 0;
	}
	if (tmpBalls > (int)MAX_BALLS) {
		tmpBalls = MAX_BALLS;
	}
	balls = tmpBalls;
}

int Collector::GetBalls()
{
	return balls;
}
