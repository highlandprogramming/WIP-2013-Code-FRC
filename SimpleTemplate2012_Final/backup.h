#include "WPILib.h"
#include "DashboardDataSender.h"
#include "DashboardDataFormat.h"
#include "Target.h"
#include "BinaryImage.h"
#include "VisionAPI.h"
#include <vector>
#include <cmath>
#include "DigitalInput.h"

											// Declaration Stage
class RobotDemo : public SimpleRobot
{
	Joystick *stick1; // Drive Joystick.
	Joystick *stick2; // Shooter Joystick.
	RobotDrive myRobot; // Robot drive system.
	Victor LoaderMotor; // Loader Motor.
	Encoder encoderR;
	Encoder encoderL;
	Gyro myGyro1; // Turn Gyro.
	Joystick *xbox;   // xbox servo control
	Compressor *compressor; // Compressor.
	Solenoid *s[8]; // Solenoids in an Array.
	PIDOutput *pidOutput; // PID Output for Dashboard.
	DashboardDataSender *dds; // Send Data to Dashboard
	Servo *servo_horiz;
	Servo *servo_vert;
	DigitalInput *lim;
	DigitalInput *distance1;
	//Gyro myGyro2; // Balance Gyro.
	//Gyro myGyro3; // Balance Gyro 1.
	
public:
																	
	RobotDemo(void):												// Initialization Stage
		myRobot(1,3,2,4),	// these must be initialized in the same order(test bed = 1 and 2 and robot = 1 2 3 and 4)
		LoaderMotor(5),
		encoderR(3,4),
		encoderL(7,8),
		myGyro1(1)
		//myGyro2(2),
		//myGyro3(3)
	{
		stick1= new Joystick(1); // Drive joystick.
		stick2= new Joystick(2); // Shooter Joystick.
		xbox = new Joystick(3); // xbox servo control
		compressor = new Compressor(1, 1); // Compressor initilization.
		s[0]= new Solenoid(1); // Solenoid Port 1 : Shooter (Piston out)
		s[1]= new Solenoid(2); // Solenoid Port 2 : Loader.
		s[2]= new Solenoid(3); // Solenoid Port 3 : Piston in
		s[3]= new Solenoid(4); // Solenoid Port 4 : None
		s[4]= new Solenoid(5); // Solenoid Port 5 : None
		s[5]= new Solenoid(6); // Solenoid Port 6 : None
		s[6]= new Solenoid(7); // Solenoid Port 7 : Left Supershifter
		s[7]= new Solenoid(8); // Solenoid Port 8 : Right Supershifter.
		dds = new DashboardDataSender(); // Opens Dashboard Data Sender Connection.
		servo_horiz = new Servo(10); // Replace 1 with the actual channel.
		servo_vert = new Servo(9); // Replace 2 with the actual channel.
		lim = new DigitalInput(2);
		distance1 = new DigitalInput(5);
		myRobot.SetExpiration(0.1); // Set Fireware Watchdog.jager was here:0
	}
	~RobotDemo()													//Clear memory
	{
		delete distance1;
		delete lim;
		delete servo_vert;
		delete servo_horiz;
		delete dds;
		delete s[7];
		delete s[6];
		delete s[5];
		delete s[4];
		delete s[3];
		delete s[2];
		delete s[1];
		delete s[0];
		delete compressor;
		delete xbox;
		delete stick2;
		delete stick1;
	}
	void RobotInit(void)											//Gyro Reset in Init
	{
		myGyro1.SetSensitivity(0.007);
		myGyro1.Reset();
	}
	void mtdEncoderForward(int distance, double turn, double speed)	//For Encoder
	{
		encoderR.Reset();
		encoderL.Reset();
		while(encoderR.GetRaw()<=distance)
		{
			if(encoderR.GetRaw()==encoderL.GetRaw())
			{
				myRobot.Drive(speed, 0.0);
			}
			else if(encoderR.GetRaw()>encoderL.GetRaw())
			{
				myRobot.Drive(speed, turn);
			}
			else if(encoderL.GetRaw()>encoderR.GetRaw())
			{	
				myRobot.Drive(speed, -turn);
			}
		}
		encoderR.Reset();
		encoderL.Reset();
	}
	void mtdEncoderTurn(int turn, double speed)						//For Encoder
	{
		encoderR.Reset();
		while(encoderR.GetRaw() <= turn)
		{
			myRobot.Drive(speed, 1.0);
		}
		encoderR.Reset();
	}
	bool blnTurnAround(float Starting, float Current)				//For Gyro
	{
		if(Starting < 0)
		{
			Starting = Starting * -1;
		}
		if(Current < 0)
		{
			Current = Current * -1;
		}
		GetWatchdog().Feed();
		if((Current - Starting) >= 120)
		{
			return true;
		}
		else
		{
			GetWatchdog().Feed();
			return false;
		}
	}
	void Autonomous(void)											//Autonomous Code
	{
		compressor->Start();
		GetWatchdog().SetEnabled(true);
		GetWatchdog().SetExpiration(10);
		GetWatchdog().Feed();
		myGyro1.SetSensitivity(0.007);
		GetWatchdog().Feed();
		myGyro1.Reset();
		GetWatchdog().Feed();
		DriverStationLCD  *dsLCD = DriverStationLCD::GetInstance();
		dsLCD->Clear();
		dsLCD->UpdateLCD();
		GetWatchdog().Feed();
		//myGyro2.SetSensitivity(0.007);
		//GetWatchdog().Feed();
		//myGyro2.Reset();
		//GetWatchdog().Feed();
		//myGyro3.SetSensitivity(0.007);
		//GetWatchdog().Feed();
		//myGyro3.Reset();
		//GetWatchdog().Feed();
		AxisCamera &camera = AxisCamera::GetInstance("10.26.3.11"); // Set Cam.IP:Router:10.26.3.11
																	//Set C-Rio:192.168.0.90
		camera.WriteResolution(AxisCamera::kResolution_320x240);	
		camera.WriteCompression(20);
		camera.WriteBrightness(0);
		GetWatchdog().Feed();
		dsLCD->UpdateLCD();
		Timer ShootTimer;
		Timer WaitTimer;
		Timer TurnTimer;
		Timer BackTimer;
		WaitTimer.Start();
		WaitTimer.Reset();
		TurnTimer.Start();
		TurnTimer.Reset();
		while(WaitTimer.Get() < 5)									//Wait.
		{
			GetWatchdog().Feed();
		}		
		s[2]->Set(true);											//Fire.
		GetWatchdog().Feed();
		Wait(0.41);
		bool blnCanShoot = true;
		ShootTimer.Start();
		ShootTimer.Reset();
		while(ShootTimer.Get() < 7)
		{
			if(blnCanShoot and ShootTimer.Get() < 0.09)
			{
				s[0]->Set(true);									//Vacuum piston in.
				s[2]->Set(false);									//Turn off fire piston.
				blnCanShoot = false;
				GetWatchdog().Feed();
			}
			if((ShootTimer.Get() > 1.65) and (ShootTimer.Get() < 1.99))
			{
				WaitTimer.Reset();
				s[0]->Set(false);									//Turn off vacuum.
				LoaderMotor.Set(-1);								//Run belt.
				GetWatchdog().Feed();
			}
			if(ShootTimer.Get() > 2.85)
			{
				LoaderMotor.Set(0);									//Belt off.
				GetWatchdog().Feed();
			}
		}
		WaitTimer.Reset();
		bool blnpointless = true;
		bool blncanTurnAround = false;
		dsLCD->UpdateLCD();
		while(IsAutonomous())
		{
			GetWatchdog().Feed();
			if (blnpointless)
			{
				blnCanShoot = true;
				s[2]->Set(true);									//Fire.
				GetWatchdog().Feed();
				blnpointless = false;
			}
			if(WaitTimer.Get() > 0.5 and blnCanShoot)
			{
				s[0]->Set(true);									//Vacuum on.
				s[2]->Set(false);									//Fire off.
				GetWatchdog().Feed();
				blnCanShoot = false;
			}
			if(WaitTimer.Get() > 1.65)
			{
				s[0]->Set(false);									//Vacuum off.
				GetWatchdog().Feed();
				blncanTurnAround = true;
				
			}
			if(blncanTurnAround)
			{
				int timecount = 0;
				while(y<12)											//Backwards 1.2 secs.
				{
					GetWatchdog().Feed();
					myRobot.Drive(.6,0);							
					Wait(.1);
					timecount++;
				}
				myRobot.Drive(0,0);									//Stop.
				float StartingAngle = myGyro1.GetAngle();			//Gyro angle fix.
				dsLCD->Clear();
				dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "%f", myGyro1.GetAngle());
				while(!blnTurnAround(StartingAngle, myGyro1.GetAngle()))
				{
					GetWatchdog().Feed();
					myRobot.Drive(.7, -1);
					dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "%f", myGyro1.GetAngle());
					dsLCD->UpdateLCD();
				}
				myRobot.Drive(0,0);									//Stop.
				break;
				int 2ndcount = 0;
				if(2ndcount == 0)
				{
					myRobot.Drive(0,0);								//Stop(again blum).
				}
				myRobot.Drive(0,0);									//Stop(again blum O.o).
			}
		}
	}
	void OperatorControl(void)										//Tele-Operated code.
	{
		myRobot.SetSafetyEnabled(true);
		GetWatchdog().SetEnabled(true);
		GetWatchdog().SetExpiration(2);
		GetWatchdog().Feed();
		compressor->Start();
		//myGyro1->Reset();
		//myGyro1.SetSensitivity(0.007);
		//myGyro1.SetAngle() = 0;
		//myGyro1.Reset();
		//myGyro2.SetSensitivity(0.007);
		//myGyro2.Reset();
		//myGyro3.SetSensitivity(0.007);
		//myGyro3.Reset();
		float Gyro1Angle;//, Gyro2Angle;//, Gyro3Angle;
		//Gyro1Angle = myGyro1.GetAngle();
		//Gyro2Angle = myGyro2.GetAngle();
		//Gyro3Angle = myGyro3.GetAngle();
		dds->sendIOPortData();
		ParticleFilterCriteria2 criteria[] = {{IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false, false},
											  {IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false, false}};
		AxisCamera &camera = AxisCamera::GetInstance("10.26.3.11"); // Set camera IP: Through Router: 10.26.3.11, Through C-Rio: 192.168.0.90
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(20);
		camera.WriteBrightness(0);
		DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();
		Timer ShootHighTimer;
		Timer ShootMidTimer;
		Timer ShootLowTimer;
		Timer PickupTimer;
		bool blnPickupSafty;
		blnPickupSafty = false;
		ShootHighTimer.Start();
		ShootMidTimer.Start();
		ShootLowTimer.Start();
		int Shoot = 0;
		bool Ramp1 = false;
		bool Ramp2 = false;
		bool blnCameraLock = true;
		bool blnBalancePosition = false;
		PickupTimer.Start();
		while (IsOperatorControl())
		{
			myRobot.ArcadeDrive(stick1); // Joystick Drive Control
			GetWatchdog().Feed();
		
			
		
			// Z Axis Servo Control (Up and down Only)
			float stick2z = -(stick2->GetZ());
			float stick1z = -(stick1->GetZ());
			stick2z = ((stick2z + 1) /2);
			stick1z = ((stick1z + 1) /2);
			float vertSet, horzSet;
			if(stick2z < 0.2)
			{
				vertSet = 0.2;
			}
			else if(stick2z > 0.9)
			{
				vertSet = (0.9);
			}
			else
			{
				vertSet = (stick2z);
			}
			if(stick1z < 0.1)
			{
				horzSet = 0.1;
			}
			else if(stick1z > 0.9)
			{
				horzSet = (0.9);
			}
			else
			{
				horzSet = (stick1z);
			}
			if(!blnCameraLock and !blnBalancePosition)
			{
				servo_horiz->Set(horzSet);
			}
			else if(blnCameraLock and !blnBalancePosition)
			{
				servo_horiz->SetAngle(89);
			}
			else if(blnBalancePosition)
			{
				servo_horiz->SetAngle(60.5);
				servo_vert->SetAngle(61);
			}
			if(!blnBalancePosition)
			{
				servo_vert->Set(vertSet);
			}
				
			GetWatchdog().Feed();

			Gyro1Angle = myGyro1.GetAngle();
			//Gyro2Angle = myGyro2.GetAngle();
			
			GetWatchdog().Feed();
			
			dsLCD->Clear();
			dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "CanShoot: %s", (lim->Get() ? ("Yes"): ("No")));
			dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "CameraLock: %s", (blnCameraLock ? ("On"): ("Off")));
			dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "BalancePos: %s", (blnBalancePosition ? ("On"): ("Off")));
			dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Ball ?: %s", (distance1->Get() ? ("No"): ("Yes")));
			dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Angle (Gyro1): %f", Gyro1Angle);
			dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Bridge Down: %s", (s[4]->Get() ? ("Down"): ("Up")));
			dsLCD->UpdateLCD();
			GetWatchdog().Feed();
			if(stick1->GetRawButton(2) && Ramp1 == false) //Get into position.
			{
				s[3]->Set(true);
				Ramp1 = true;
				Wait(.4);
			}
			
			else if(stick1->GetRawButton(2) && Ramp1 == true/* && Ramp2 == false*/) //Put away
			{
				s[3]->Set(false);
				Ramp1 = false;
				Wait(.4);
			}
			if(stick1->GetRawButton(3)/* && Ramp1 == true*/ && Ramp2 == false) //Shoot out
			{
				s[4]->Set(true);
				Ramp2 = true;
				Wait(.4);
			}
			else if(stick1->GetRawButton(3) && Ramp2 == true) //Retract
			{
				s[4]->Set(false);
				Ramp2 = false;
				Wait(.4);
			}
			if(stick2->GetRawButton(10))
			{
				blnCameraLock ? (blnCameraLock = false): (blnCameraLock = true);
				Wait(.4);
				GetWatchdog().Feed();
			}
			if(stick2->GetRawButton(9))
			{
				GetWatchdog().Feed();
				ColorImage *image;
				image = camera.GetImage();
				BinaryImage *thresholdImage = image->ThresholdHSL(0, 255, 0, 255, 160, 255);	// get just the red target pixels
				BinaryImage *bigObjectsImage = thresholdImage->RemoveSmallObjects(false, 2);  // remove small objects (noise)
				BinaryImage *convexHullImage = bigObjectsImage->ConvexHull(false);  // fill in partial and full rectangles
				BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 2);  // find the rectangles
				vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  // get the results
				GetWatchdog().Feed();
				
				ParticleAnalysisReport& par = (*reports)[0];
				int pLength = par.boundingRect.width;
				
				//pLength = 24;
				double distance = (320/pLength)/(0.509525);
				GetWatchdog().Feed();
				dsLCD->Clear();
				dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Distance12: %d", pLength);
				dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Distance45: %f", distance);
				dsLCD->UpdateLCD();
				GetWatchdog().Feed();
				delete reports;
				delete filteredImage;
				delete convexHullImage;
				delete bigObjectsImage;
				delete thresholdImage;
				delete image;
				GetWatchdog().Feed();
				Wait(0.1);
			}
			if(stick1->GetRawButton(6)) // High-Gear
			{
				GetWatchdog().Feed();
				s[6]->Set(true);
				s[7]->Set(false);
				GetWatchdog().Feed();
			}
			if(stick1->GetRawButton(7)) // Low-Gear
			{
				GetWatchdog().Feed();
				s[6]->Set(false);
				s[7]->Set(true);
				GetWatchdog().Feed();
			}
			if(stick2->GetRawButton(6))// && blnPickupSafty == false) // Loader: On
			{
				GetWatchdog().Feed();
				LoaderMotor.Set(-1);
				GetWatchdog().Feed();
			}
			if(stick2->GetRawButton(7)/* && blnPickupSafty == false*/) // Loader: Off
			{
				GetWatchdog().Feed();
				LoaderMotor.Set(0);
				GetWatchdog().Feed();
			}
			if(stick2->GetRawButton(8)/* && blnPickupSafty == false*/) //Loader: Reverse
			{
				GetWatchdog().Feed();
				LoaderMotor.Set(1);
				GetWatchdog().Feed();
			}
			if(stick2->GetRawButton(11))
			{
				blnBalancePosition ? (blnBalancePosition = false): (blnBalancePosition = true);
				Wait(.4);
				GetWatchdog().Feed();
			}
			if((PickupTimer.Get() > 3.5) && blnPickupSafty == true)
			{
				GetWatchdog().Feed();
				LoaderMotor.Set(0);
				blnPickupSafty = false;
				GetWatchdog().Feed();
			}
			if(stick2->GetTrigger() && (ShootHighTimer.Get() > 1.0)) // Shoot
			{
				s[2]->Set(true);
				GetWatchdog().Feed();
				GetWatchdog().Feed();
				ShootHighTimer.Reset();
				Shoot = 1;
			}
			else if(Shoot == 1 && ShootHighTimer.Get() > 0.5 && ShootHighTimer.Get() < 1.65)//1.1
			{
				s[0]->Set(true);
				GetWatchdog().Feed();
				s[2]->Set(false);
				GetWatchdog().Feed();
			}
			else if(Shoot == 1 && ShootHighTimer.Get() > 1.65)//2.2
			{
				GetWatchdog().Feed();
				s[0]->Set(false);
				GetWatchdog().Feed();
				Shoot = 0;
			}
			if(stick2->GetRawButton(3) && (ShootMidTimer.Get() > 3.0)) // Shoot
			{
				s[2]->Set(true);
				GetWatchdog().Feed();
				ShootMidTimer.Reset();
				Shoot = 2;
			}
			else if(Shoot == 2 && ShootMidTimer.Get() > 0.06 && ShootMidTimer.Get() < 1.21)//1.1
			{
				s[0]->Set(true);
				GetWatchdog().Feed();
				s[2]->Set(false);
				GetWatchdog().Feed();
			}
			else if(Shoot == 2 && ShootMidTimer.Get() > 1.21)//2.2
			{
				GetWatchdog().Feed();
				s[0]->Set(false);
				GetWatchdog().Feed();
				Shoot = 0;
			}
			if(stick2->GetRawButton(2) && (ShootLowTimer.Get() > 3.0)) // Shoot
			{
				s[2]->Set(true);
				GetWatchdog().Feed();
				ShootLowTimer.Reset();
				Shoot = 3;
			}
			else if(Shoot == 3 && ShootLowTimer.Get() > 0.05 && ShootLowTimer.Get() < 1.2)//1.1
			{
				s[0]->Set(true);
				GetWatchdog().Feed();
				s[2]->Set(false);
				GetWatchdog().Feed();
			}
			else if(Shoot == 3 && ShootLowTimer.Get() > 1.2)//2.2
			{
				GetWatchdog().Feed();
				s[0]->Set(false);
				GetWatchdog().Feed();
				Shoot = 0;
			}
			if(stick2->GetRawButton(4) && (ShootLowTimer.Get() > 3.0)) // Shoot
			{
				s[2]->Set(true);
				GetWatchdog().Feed();
				ShootLowTimer.Reset();
				Shoot = 4;
			}
			else if(Shoot == 4 && ShootLowTimer.Get() > 0.02 && ShootLowTimer.Get() < 1.17)//1.1
			{
				s[0]->Set(true);
				GetWatchdog().Feed();
				s[2]->Set(false);
				GetWatchdog().Feed();
			}
			else if(Shoot == 4 && ShootLowTimer.Get() > 1.17)//2.2
			{
				GetWatchdog().Feed();
				s[0]->Set(false);
				GetWatchdog().Feed();
				Shoot = 0;
			}
			if(stick2->GetRawButton(5) && (ShootLowTimer.Get() > 3.0)) // Shoot
			{
				s[2]->Set(true);
				GetWatchdog().Feed();
				ShootLowTimer.Reset();
				Shoot = 3;
			}
			else if(Shoot == 5 && ShootLowTimer.Get() > 0.01 && ShootLowTimer.Get() < 1.16)//1.1
			{
				s[0]->Set(true);
				GetWatchdog().Feed();
				s[2]->Set(false);
				GetWatchdog().Feed();
			}
			else if(Shoot == 5 && ShootLowTimer.Get() > 1.16)//2.2
			{
				GetWatchdog().Feed();
				s[0]->Set(false);
				GetWatchdog().Feed();
				Shoot = 0;
			}
			if(stick1->GetRawButton(4))
			{
				myRobot.Drive(-0.3, 0.0);
			}	
			if(stick1->GetRawButton(5))
			{
				myRobot.Drive(0.3, 0.0);
			}
			/*if(stick1->GetTrigger())
			{
				GetWatchdog().Feed();
				float fltBalance, fltBalance1, fltBalance2, fltBalTot;
				//myGyro2.Reset();
				GetWatchdog().Feed();
				//myGyro3.Reset();
				GetWatchdog().Feed();
				fltBalance1 = myGyro2.GetAngle();
				GetWatchdog().Feed();
				fltBalance2 = myGyro3.GetAngle();
				GetWatchdog().Feed();
				fltBalTot = fltBalance1 + fltBalance2;
				GetWatchdog().Feed();
				fltBalance = fltBalTot/2;
				GetWatchdog().Feed();

				if(fltBalance > 4)
				{
					GetWatchdog().Feed();
					myRobot.Drive(0.3, 0.0);
					GetWatchdog().Feed();
				}
				else if(fltBalance <= 4 && fltBalance >= -4)
				{
					GetWatchdog().Feed();
					myRobot.Drive(0,0);
					GetWatchdog().Feed();
				}
				else if(fltBalance < -4)
				{
					GetWatchdog().Feed();
					myRobot.Drive(-0.3, 0.0);
					GetWatchdog().Feed();
				}
			}*/
		}
	}
};

START_ROBOT_CLASS(RobotDemo);