#include "WPILib.h"
#include "Dashboard.h"
#include "DashboardBase.h"
#include <vector>
#include <cmath>
#include "DigitalInput.h"
#include "SmartDashboard/SmartDashboard.h"
//#include "SmartDashboard/SendableChooser.h"
//#include "SmartDashboard/Sendable.h"
//#include "SmartDashboard/NamedSendable.h"
//#include "Target.h"
//#include "BinaryImage.h"
//#include "VisionAPI.h"

class RobotDemo : public SimpleRobot
{
	//Declare-----------
	//SmartDashboard *sd;
	RobotDrive myRobot;
	Joystick *stick1;
	Joystick *stick2;
	Compressor *compressor;
	Solenoid *s[8];
	PIDOutput *pidOutput;
	
public:
	RobotDemo(void):
		myRobot(1,3,2,4)			
	{
		//Init-----------
		stick1 = new Joystick(1);
		stick2 = new Joystick(2);
		compressor = new Compressor(1,1);
		s[0] = new Solenoid(1);
		s[1] = new Solenoid(2);
		s[2] = new Solenoid(3);
		s[3] = new Solenoid(4);
		s[4] = new Solenoid(5);
		s[5] = new Solenoid(6);
		s[6] = new Solenoid(7);
		s[7] = new Solenoid(8);
		SmartDashboard::init();
		myRobot.SetExpiration(0.1);
	}
	
	//Deconstructor
	~RobotDemo()
	{
		delete s[7];
		delete s[6];
		delete s[5];
		delete s[4];
		delete s[3];
		delete s[2];
		delete s[1];
		delete s[0];
		delete compressor;
		delete stick2;
		delete stick1;
		
	}
	
	void RobotInit(void)
	{
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->UpdateLCD();
	}

	void Autonomous(void)
	{
		compressor->Start();
		GetWatchdog().SetEnabled(true);
		GetWatchdog().SetExpiration(10);
		GetWatchdog().Feed();
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(0.5, 0.0); 	
		Wait(2.0); 				
		myRobot.Drive(0.0, 0.0); 	
	}

	void OperatorControl(void)
	{
		compressor->Start();
		myRobot.SetSafetyEnabled(true);
		GetWatchdog().SetEnabled(true);
		GetWatchdog().SetExpiration(8);
		GetWatchdog().Feed();
		//sd->sendIOPortData();
		while (IsOperatorControl())
		{
			myRobot.ArcadeDrive(stick1);
			SmartDashboard::PutNumber("demo",1);
			GetWatchdog().Feed();
		}
	}
};

START_ROBOT_CLASS(RobotDemo);