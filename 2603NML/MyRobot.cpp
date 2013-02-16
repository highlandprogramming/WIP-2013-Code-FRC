#include "WPILib.h"
#include "Dashboard.h"
#include "DashboardBase.h"
#include <vector>
#include <cmath>
#include "DigitalInput.h"
#include "SmartDashboard/SmartDashboard.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"
#include "myRobotDrive.h"

//Camera constants used for distance calculation
#define X_IMAGE_RES 320		//X Image resolution in pixels, should be 160, 320 or 640
//#define VIEW_ANGLE 48		//Axis 206 camera
#define VIEW_ANGLE 43.5  //Axis M1011 camera
#define PI 3.141592653

//Score limits used for target identification
#define RECTANGULARITY_LIMIT 60
#define ASPECT_RATIO_LIMIT 75
#define X_EDGE_LIMIT 40
#define Y_EDGE_LIMIT 60

//Minimum area of particles to be considered
#define AREA_MINIMUM 500

//Edge profile constants used for hollowness score calculation
#define XMAXSIZE 24
#define XMINSIZE 24
#define YMAXSIZE 24
#define YMINSIZE 48
const double xMax[XMAXSIZE] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
const double xMin[XMINSIZE] = {.4, .6, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, 0.6, 0};
const double yMax[YMAXSIZE] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
const double yMin[YMINSIZE] = {.4, .6, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
								.05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
								.05, .05, .6, 0};


class RobotDemo : public SimpleRobot
{
	
	struct Scores
	{
		double rectangularity;
		double aspectRatioInner;
		double aspectRatioOuter;
		double xEdge;
		double yEdge;

	};
	
	//Declare-----------
	Talon *LF;
	Talon *LR;
	Talon *RF;
	Talon *RR;
	RobotDrive myRobot;
	Victor myShooter1;
	Victor myShooter2;
	Joystick *stick1;
	Joystick *stick2;
	Joystick *x360;
	Compressor *compressor;
	Solenoid *s[8];
	PIDOutput *pidOutput;
	Scores *scores;
//	char particle[];

	
	
public:
	RobotDemo(void):
		
		//myRobot(1,3,2,4),
		myRobot(LF,LR,RF,RR),
		myShooter1(5),
		myShooter2(6)
	{
		//Init-----------
		LF = new Talon(1);
		LR = new Talon(3);
		RF = new Talon(2);
		RR = new Talon(4);
		stick1 = new Joystick(1);
		stick2 = new Joystick(2);
		x360 = new Joystick(3);
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
		//blnShift = true;
		}

	void Autonomous(void)
	{
		//s[0]->Set(true);
		//s[1]->Set(true);
		SmartDashboard::PutString("Gear","High");
		//compressor->Start();
		GetWatchdog().SetEnabled(false);
		//GetWatchdog().SetExpiration(10);
		//GetWatchdog().Feed();
		myRobot.SetSafetyEnabled(false);
		/*myRobot.Drive(0.5, 0.0); 	
		Wait(2.0); 				
		myRobot.Drive(0.0, 0.0); 	
		*/
		
		//int x = 0;
		
		int WaitDash = 0;
		int FireDash = 0;
		int intPause = 0;
		
		//SmartDashboard::PutNumber("fire_wait", 3);
		//SmartDashboard::PutNumber("fire_amount", 3);
		//SmartDashboard::PutNumber("fire_pause", 5);
		
		WaitDash = static_cast<int>(SmartDashboard::GetNumber("fire_wait"));
		FireDash = static_cast<int>(SmartDashboard::GetNumber("fire_amount"));
		intPause = static_cast<int>(SmartDashboard::GetNumber("fire_pause"));
		

		
		//Their threshold values suck DDDD, from the NIvision assistant will be below.
		//Threshold threshold(60, 100, 90, 255, 20, 255);	//HSV threshold criteria, ranges are in that order ie. Hue is 60-100
		//Threshold threshold(100, 255, 230, 255, 140, 255); //New threshold values
		Threshold threshold(0, 255, 0, 255, 221, 255);
		
		ParticleFilterCriteria2 criteria[] = {
				{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}
		};												//Particle filter criteria, used to filter out small particles
		// AxisCamera &camera = AxisCamera::GetInstance();	//To use the Axis camera uncomment this line
				
		while (IsAutonomous() && IsEnabled())
		{
		          /**
		             * Do the image capture with the camera and apply the algorithm described above. This
		             * sample will either get images from the camera or from an image file stored in the top
		             * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
		             */
			//SmartDashboard::PutNumber("Test", 1);
			//if(0 == 0)
			//{
			
			compressor->Enabled();
			compressor->Start();
			
			myRobot.SetSafetyEnabled(true);
			
			
			GetWatchdog().SetEnabled(true);
			GetWatchdog().SetExpiration(1);
			GetWatchdog().Feed();
			
			myShooter1.Set(-1);
			myShooter2.Set(-1);
			GetWatchdog().Feed();
		
			
			GetWatchdog().Feed();
			
			// Wait before firing.
			for(int x = 0; x< ((intPause*2)+1);x++)
			{
				GetWatchdog().Feed();
				Wait(0.5);
			}

			
			// Fire a number of frisbees as set in the dashboard.
			for(int count = 0; count < (FireDash+1);count++)
			{
				GetWatchdog().Feed();
				s[2]->Set(true);
				
				// Delay between firing each frisbees.
				for(int wait = 0; wait<(WaitDash);wait++)
				{
					GetWatchdog().Feed();
					Wait(0.5);
				}
				s[2]->Set(false);
				GetWatchdog().Feed();
				for(int secondwait = 0;secondwait<3;secondwait++)
				{
					GetWatchdog().Feed();
					Wait(0.5);
				}
			}
			
			
			myShooter1.Set(0);
			myShooter2.Set(0);
			GetWatchdog().Feed();
			
			//SmartDashboard::PutNumber("Test", 2);
			AxisCamera &camera = AxisCamera::GetInstance("10.26.3.11");
			camera.WriteResolution(AxisCamera::kResolution_320x240);
			camera.WriteCompression(20);
			camera.WriteBrightness(50);
			

			//SmartDashboard::PutNumber("Test", 3);
			
			ColorImage *image;
			//image = new RGBImage("/HybridLine_DoubleGreenBK3.jpg");		// get the sample image from the cRIO flash
			image = camera.GetImage();
					//camera.GetImage(image);				//To get the images from the camera comment the line above and uncomment this one
			Wait(.1);
			
			//SmartDashboard::PutNumber("Test", 4);
			BinaryImage *thresholdImage = image->ThresholdHSV(threshold);	// get just the green target pixels
			//		thresholdImage->Write("/threshold.bmp");

			//SmartDashboard::PutNumber("Test", 5);
			BinaryImage *convexHullImage = thresholdImage->ConvexHull(false);  // fill in partial and full rectangles
		//			convexHullImage->Write("ConvexHull.bmp");

			//SmartDashboard::PutNumber("Test", 6);
			BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 1);	//Remove small particles
			//		filteredImage->Write("/Filtered.bmp");

			//SmartDashboard::PutNumber("Test", 7);
			vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  //get a particle analysis report for each particle

			//SmartDashboard::PutNumber("Test", 8);
			int size = reports->size();
			scores = new Scores[size];
					

			//SmartDashboard::PutNumber("Test", 9);
					//Iterate through each particle, scoring it and determining whether it is a target or not
			for (unsigned i = 0; i < reports->size(); i++)
			{

				//SmartDashboard::PutNumber("Test", 10);
				ParticleAnalysisReport *report = &(reports->at(i));
						
				scores[i].rectangularity = scoreRectangularity(report);
				scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage, report, true);
				scores[i].aspectRatioInner = scoreAspectRatio(filteredImage, report, false);			
				scores[i].xEdge = scoreXEdge(thresholdImage, report);
				scores[i].yEdge = scoreYEdge(thresholdImage, report);
				
						
				if(scoreCompare(scores[i], false))
				{
					//We hit this!! Note to self: changethe below printf statement
					//To use SmartDashboard::PutString so wecan seevalues.
					//printf("particle: %d  is a High Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
					//string particle = ("particle: %d  is a High Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
					
					SmartDashboard::PutNumber("CenterX", report->center_mass_x);
					SmartDashboard::PutNumber("CenterY", report->center_mass_y);
					SmartDashboard::PutNumber("Area", report->particleArea);
					SmartDashboard::PutNumber("Distance",computeDistance(thresholdImage,report, false));
					SmartDashboard::PutNumber("size", reports->size());
					SmartDashboard::PutNumber("height", report->boundingRect.height);
					SmartDashboard::PutNumber("Quality", report->particleQuality);
					//SmartDashboard::PutNumber("Test",computeDistance(thresholdImage, report, false));
					//SmartDashboard::PutNumber("Distance",computeDistance(thresholdImage,report, false));
					SmartDashboard::PutString("high goal detected", "asdf");
				} 
				
				else if (scoreCompare(scores[i], true))
				{
					printf("particle: %d  is a Middle Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
					SmartDashboard::PutNumber("Test", computeDistance(thresholdImage, report, true));
					SmartDashboard::PutNumber("CenterX", report->center_mass_x);
					SmartDashboard::PutNumber("CenterY", report->center_mass_y);
					SmartDashboard::PutNumber("height", report->boundingRect.height);
					SmartDashboard::PutNumber("Distance",computeDistance(thresholdImage,report, false));
					SmartDashboard::PutString("middle goal detected", "adsf");
				
				}
				
				else
				{
					printf("particle: %d  is not a goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
					SmartDashboard::PutNumber("CenterX", report->center_mass_x);
					SmartDashboard::PutNumber("CenterY", report->center_mass_y);
					SmartDashboard::PutNumber("height", report->boundingRect.height);
					SmartDashboard::PutNumber("Distance",computeDistance(thresholdImage,report, false));
					SmartDashboard::PutString("we areinelse", "else");
										
				}
				if(report->center_mass_x < 85.00)
				{								
					SmartDashboard::PutString("Pausing", "paused");
					//image->Write("C:\\testimg.bmp");
					//Wait(10);
				}
				printf("rect: %f  ARinner: %f \n", scores[i].rectangularity, scores[i].aspectRatioInner);
				printf("ARouter: %f  xEdge: %f  yEdge: %f  \n", scores[i].aspectRatioOuter, scores[i].xEdge, scores[i].yEdge);	
			}
			printf("\n");
					
					// be sure to delete images after using them
			delete filteredImage;
			delete convexHullImage;
			delete thresholdImage;
			delete image;
					
					//delete allocated reports and Scores objects also
			delete scores;
			delete reports;
				//}
			//x++;
		}
		
	}

	void OperatorControl(void)
	{
		// Teleoperated Code.
		/*double WaitDash = 0.0;
		double FireDash = 0.0;
		double intPause = 0.0;
		
		SmartDashboard::PutNumber("P", 3.0);
		SmartDashboard::PutNumber("W", 0.0);
		SmartDashboard::PutNumber("A", 0.0);
		
		WaitDash = SmartDashboard::GetNumber("P");
		FireDash = SmartDashboard::GetNumber("W");
		intPause = SmartDashboard::GetNumber("A");
		
		SmartDashboard::PutNumber("P", WaitDash);
		SmartDashboard::PutNumber("W", FireDash);
		SmartDashboard::PutNumber("A", WaitDash);
		*/	
		// Enable and start the compressor.
		//compressor->Enabled();
		compressor->Start();
			
		// Enable drive motor safety timeout.
		myRobot.SetSafetyEnabled(true);
			
		// Enable watchdog and initial feed.
		GetWatchdog().SetEnabled(true);
		GetWatchdog().SetExpiration(1);
		GetWatchdog().Feed();
			
		// Set robot in low gear by default. Not active.
		//s[0]->Set(false);
		GetWatchdog().Feed();
		
		//bool blnShoot = false;
		bool blnLowHang = false;
		bool blnShift = false;

		GetWatchdog().Feed();
		
		bool blnShooterSpd = false;
		
		float fltShoot;
		float fltSpeed = 1;
		
		int intFail = 0;
		
		GetWatchdog().Feed();
			
		//sd->sendIOPortData();

		// Local variables.
		//float fltStick1X, fltStick1Y;
		
		while (IsOperatorControl())
		{
			/*
			// Gamepad tankdrive code.
			myRobot.TankDrive(x360->GetRawAxis(2),x360->GetRawAxis(4));

			fltStick1Y = (x360->GetRawAxis(2))*(-100);
			fltStick1X = (x360->GetRawAxis(4))*(-100);

			SmartDashboard::PutNumber("Left Throttle (%)",fltStick1Y);
			SmartDashboard::PutNumber("Right Throttle (%)",fltStick1X);
			// End Gamepad tankdrive code.bvg
			*/
			
			
			
			// Stick1 arcade drive code.
			//myRobot.ArcadeDrive(stick1->GetY(),0,true);
			//LF->Set(stick1->GetY());
			//LR->Set(stick1->GetY());
			//RF->Set(stick1->GetY());
			//RR->Set(stick1->GetY());
			
			singleDrive(stick1->GetY(),stick1->GetX(),true);
			
			//myRobot.ArcadeDrive(stick1);
			GetWatchdog().Feed(); // Feed hungary demonic Watchdog.

				
			SmartDashboard::PutNumber("Throttle (%)",stick1->GetY()*(-100));
			SmartDashboard::PutNumber("Steering (%)",stick1->GetX()*(100));

			GetWatchdog().Feed();
			//End Stick1 arcade drive code.

			GetWatchdog().Feed();
			
			fltShoot = (((-(stick2->GetRawAxis(3)))+1)/2);

			GetWatchdog().Feed();
			
			SmartDashboard::PutNumber("Shooter Power (%)", fltShoot);
			SmartDashboard::PutNumber("Shooter Set Speed (%)", (fltSpeed*100));
			
			//float fltPressureSwitch = m_pressureSwitch;
			//float fltRelay = m_relay;
			//SmartDashboard::PutNumber("Demo",3);
				
			GetWatchdog().Feed();
			if(stick1->GetTrigger() && blnShift == false)
			{
				
				//SafetyTimer.Reset();
				//SafetyTimer.Start();
				GetWatchdog().Feed();
				s[0]->Set(false);
				s[1]->Set(true);
				SmartDashboard::PutString("Gear","Low");
				blnShift = true;
				Wait(.5);
					
				GetWatchdog().Feed();
				
			}
			else if(stick1->GetTrigger() && blnShift == true)
			{
				
				GetWatchdog().Feed();
				s[0]->Set(true);
				s[1]->Set(false);
				SmartDashboard::PutString("Gear","High");
				blnShift = false;
				Wait(.5);
				GetWatchdog().Feed();
				
			}
			
			if(stick1->GetRawButton(2) && blnLowHang == false)
			{
				
				s[3]->Set(true);
				blnLowHang = true;
				SmartDashboard::PutString("Low Hang", "Out");
				GetWatchdog().Feed();
				
			}
			
			else if(stick1->GetRawButton(2) && blnLowHang == true)
			{
				
				s[3]->Set(false);
				blnLowHang = false;
				SmartDashboard::PutString("Low Hang", "In");
				GetWatchdog().Feed();
				
			}
			
			if(stick1->GetRawButton(3))
			{
				
				mtdCameraCode();
				GetWatchdog().Feed();
				
			}
			
			/*if(stick2->GetTrigger() && blnShoot == false)
			{
				
				s[2]->Set(true);
				SmartDashboard::PutString("Shooter Piston","In");
				blnShoot = true;
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
				
			}
			else if(stick2->GetTrigger() && blnShoot == true)
			{
				
				s[2]->Set(false);
				blnShoot = false;
				SmartDashboard::PutString("Shooter Piston","Out");
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
				
			}*/
			
			if(stick2->GetTrigger() && intFail == 0)
			{
				
				s[2]->Set(true);
				SmartDashboard::PutString("Shooter Piston","In");
				intFail = 1;
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
				
			}
			else if(stick2->GetTrigger() && intFail == 1)
			{
				
				s[2]->Set(false);
				intFail = 0;
				SmartDashboard::PutString("Shooter Piston","Out");
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
				
			}
			
			if(stick2->GetRawButton(2) && blnShooterSpd == false)
			{
				
				myShooter1.Set(-fltSpeed);
				myShooter2.Set(-fltSpeed);
				SmartDashboard::PutString("Shooter","On");
				SmartDashboard::PutNumber("Shooter Speed (%)",(fltSpeed)*(100));
				blnShooterSpd = true;
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
				
			}
			else if(stick2->GetRawButton(2) && blnShooterSpd == true)
			{
				
				myShooter1.Set(0);
				myShooter2.Set(0);
				SmartDashboard::PutString("Shooter","Off");
				SmartDashboard::PutNumber("Shooter Speed (%)",0);
				blnShooterSpd = false;
				GetWatchdog().Feed();
				Wait(0.5);
				GetWatchdog().Feed();
				
			}
			
			if(stick2->GetRawButton(10))
			{
				fltSpeed = 0.6;
				GetWatchdog().Feed();
			}
			if(stick2->GetRawButton(9))
			{
				fltSpeed = 0.7;
				GetWatchdog().Feed();
			}
			if(stick2->GetRawButton(8))
			{
				fltSpeed = 0.8;
				GetWatchdog().Feed();
			}
			if(stick2->GetRawButton(7))
			{
				fltSpeed = 0.9;
				GetWatchdog().Feed();
			}
			if(stick2->GetRawButton(6))
			{
				fltSpeed = 1;
				GetWatchdog().Feed();
			}
			if(stick2->GetRawButton(11))
			{
				fltSpeed = fltShoot;
				GetWatchdog().Feed();
			}
			GetWatchdog().Feed();
		}
	}
	
	/*
	void Test(void)
	{
		compressor->Start();
		myRobot.SetSafetyEnabled(true);
		GetWatchdog().SetEnabled(true);
		GetWatchdog().SetExpiration(1);
		GetWatchdog().Feed();
		
		while(IsTest())
		{

			GetWatchdog().Feed();
			if(stick1->GetRawButton(3))
			{
				leftFront.Set(-0.1);
				leftRear.Set(0.1);
				rightFront.Set(-0.1);
				rightRear.Set(0.1);
				GetWatchdog().Feed();
			}
			else
			{
				myRobot.ArcadeDrive(stick1);
				GetWatchdog().Feed();
			}
		}
	}
	*/
	
	void mtdCameraCode(void)
	{
		Threshold threshold(0, 255, 0, 255, 221, 255);
		
		ParticleFilterCriteria2 criteria[] = {{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}};		
	
		AxisCamera &camera = AxisCamera::GetInstance("10.26.3.11");
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(20);
		camera.WriteBrightness(50);
					

		//SmartDashboard::PutNumber("Test", 3);
					
		ColorImage *image;
		//image = new RGBImage("/HybridLine_DoubleGreenBK3.jpg");		// get the sample image from the cRIO flash
		image = camera.GetImage();
		//camera.GetImage(image);				//To get the images from the camera comment the line above and uncomment this one
		Wait(.1);
					
		//SmartDashboard::PutNumber("Test", 4);
		BinaryImage *thresholdImage = image->ThresholdHSV(threshold);	// get just the green target pixels
		//		thresholdImage->Write("/threshold.bmp");

		//SmartDashboard::PutNumber("Test", 5);
		BinaryImage *convexHullImage = thresholdImage->ConvexHull(false);  // fill in partial and full rectangles
		//			convexHullImage->Write("ConvexHull.bmp");

		//SmartDashboard::PutNumber("Test", 6);
		BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 1);	//Remove small particles
		//		filteredImage->Write("/Filtered.bmp");
		//SmartDashboard::PutNumber("Test", 7);
		vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  //get a particle analysis report for each particle

		//SmartDashboard::PutNumber("Test", 8);
		int size = reports->size();
		scores = new Scores[size];
							

		//SmartDashboard::PutNumber("Test", 9);
		//Iterate through each particle, scoring it and determining whether it is a target or not
		for (unsigned i = 0; i < reports->size(); i++)
		{
			//SmartDashboard::PutNumber("Test", 10);
			ParticleAnalysisReport *report = &(reports->at(i));
								
			scores[i].rectangularity = scoreRectangularity(report);
			scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage, report, true);
			scores[i].aspectRatioInner = scoreAspectRatio(filteredImage, report, false);			
			scores[i].xEdge = scoreXEdge(thresholdImage, report);
			scores[i].yEdge = scoreYEdge(thresholdImage, report);
						
								
			if(scoreCompare(scores[i], false))
			{
				//We hit this!! Note to self: changethe below printf statement
				//To use SmartDashboard::PutString so wecan seevalues.
				//printf("particle: %d  is a High Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
				//string particle = ("particle: %d  is a High Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
							
				SmartDashboard::PutNumber("CenterX", report->center_mass_x);
				SmartDashboard::PutNumber("CenterY", report->center_mass_y);
				SmartDashboard::PutNumber("Area", report->particleArea);
				SmartDashboard::PutNumber("Distance",computeDistance(thresholdImage,report, false));
				SmartDashboard::PutNumber("size", reports->size());
				SmartDashboard::PutNumber("height", report->boundingRect.height);
				SmartDashboard::PutNumber("Quality", report->particleQuality);
				//SmartDashboard::PutNumber("Test",computeDistance(thresholdImage, report, false));
				//SmartDashboard::PutNumber("Distance",computeDistance(thresholdImage,report, false));
				SmartDashboard::PutString("high goal detected", "asdf");
			} 
						
			else if (scoreCompare(scores[i], true))
			{
				printf("particle: %d  is a Middle Goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
				SmartDashboard::PutNumber("Test", computeDistance(thresholdImage, report, true));
				SmartDashboard::PutNumber("CenterX", report->center_mass_x);
				SmartDashboard::PutNumber("CenterY", report->center_mass_y);
				SmartDashboard::PutNumber("height", report->boundingRect.height);
				SmartDashboard::PutNumber("Distance",computeDistance(thresholdImage,report, false));
				SmartDashboard::PutString("middle goal detected", "adsf");
				
			}
						
			else
			{
				printf("particle: %d  is not a goal  centerX: %f  centerY: %f \n", i, report->center_mass_x_normalized, report->center_mass_y_normalized);
				SmartDashboard::PutNumber("CenterX", report->center_mass_x);
				SmartDashboard::PutNumber("CenterY", report->center_mass_y);
				SmartDashboard::PutNumber("height", report->boundingRect.height);
				SmartDashboard::PutNumber("Distance",computeDistance(thresholdImage,report, false));
				SmartDashboard::PutString("we areinelse", "else");
									
			}
			if(report->center_mass_x < 85.00)
			{								
				SmartDashboard::PutString("Pausing", "paused");
				//image->Write("C:\\testimg.bmp");
				//Wait(10);
			}
			printf("rect: %f  ARinner: %f \n", scores[i].rectangularity, scores[i].aspectRatioInner);
			printf("ARouter: %f  xEdge: %f  yEdge: %f  \n", scores[i].aspectRatioOuter, scores[i].xEdge, scores[i].yEdge);	
		}
		printf("\n");
							
		// be sure to delete images after using them
		delete filteredImage;
		delete convexHullImage;
		delete thresholdImage;
		delete image;
							
		//delete allocated reports and Scores objects also
		delete scores;
		delete reports;
	}
	
	double computeDistance (BinaryImage *image, ParticleAnalysisReport *report, bool outer) {
		double rectShort, height;
		int targetHeight;
		
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
		//using the smaller of the estimated rectangle short side and the bounding rectangle height results in better performance
		//on skewed rectangles
		SmartDashboard::PutNumber("rectShort", rectShort);
		//height = min(report->boundingRect.height, rectShort);
		height = report->boundingRect.height;
		targetHeight = outer ? 29 : 21;
		
		return X_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
	}
	
	double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool outer){
		double rectLong, rectShort, idealAspectRatio, aspectRatio;
		idealAspectRatio = outer ? (62/29) : (62/20);	//Dimensions of goal opening + 4 inches on all 4 sides for reflective tape
		
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
		
		//Divide width by height to measure aspect ratio
		if(report->boundingRect.width > report->boundingRect.height){
			//particle is wider than it is tall, divide long by short
			aspectRatio = 100*(1-fabs((1-((rectLong/rectShort)/idealAspectRatio))));
		} else {
			//particle is taller than it is wide, divide short by long
			aspectRatio = 100*(1-fabs((1-((rectShort/rectLong)/idealAspectRatio))));
		}
		return (max(0, min(aspectRatio, 100)));		//force to be in range 0-100
	}
	
	bool scoreCompare(Scores scores, bool outer){
		bool isTarget = true;

		isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
		if(outer){
			isTarget &= scores.aspectRatioOuter > ASPECT_RATIO_LIMIT;
		} else {
			isTarget &= scores.aspectRatioInner > ASPECT_RATIO_LIMIT;
		}
		isTarget &= scores.xEdge > X_EDGE_LIMIT;
		isTarget &= scores.yEdge > Y_EDGE_LIMIT;

		return isTarget;
	}
	
	double scoreRectangularity(ParticleAnalysisReport *report){
		if(report->boundingRect.width*report->boundingRect.height !=0){
			return 100*report->particleArea/(report->boundingRect.width*report->boundingRect.height);
		} else {
			return 0;
		}	
	}
	
	double scoreXEdge(BinaryImage *image, ParticleAnalysisReport *report){
		double total = 0;
		LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(), IMAQ_COLUMN_AVERAGES, report->boundingRect);
		for(int i=0; i < (averages->columnCount); i++){
			if(xMin[i*(XMINSIZE-1)/averages->columnCount] < averages->columnAverages[i] 
			   && averages->columnAverages[i] < xMax[i*(XMAXSIZE-1)/averages->columnCount]){
				total++;
			}
		}
		total = 100*total/(averages->columnCount);		//convert to score 0-100
		imaqDispose(averages);							//let IMAQ dispose of the averages struct
		return total;
	}
	
	double scoreYEdge(BinaryImage *image, ParticleAnalysisReport *report){
		double total = 0;
		LinearAverages *averages = imaqLinearAverages2(image->GetImaqImage(), IMAQ_ROW_AVERAGES, report->boundingRect);
		for(int i=0; i < (averages->rowCount); i++){
			if(yMin[i*(YMINSIZE-1)/averages->rowCount] < averages->rowAverages[i] 
			   && averages->rowAverages[i] < yMax[i*(YMAXSIZE-1)/averages->rowCount]){
				total++;
			}
		}
		total = 100*total/(averages->rowCount);		//convert to score 0-100
		imaqDispose(averages);						//let IMAQ dispose of the averages struct
		return total;
	}	
	
	void singleDrive(float moveValue, float rotateValue, bool squaredInputs)
	{

		// local variables to hold the computed PWM values for the motors
		float leftMotorOutput;
		float rightMotorOutput;

		moveValue = Limit(moveValue);
		rotateValue = Limit(rotateValue);

		if (squaredInputs)
		{
			// square the inputs (while preserving the sign) to increase fine control while permitting full power
			if (moveValue >= 0.0)
			{
				moveValue = (moveValue * moveValue);
			}
			else
			{
				moveValue = -(moveValue * moveValue);
			}
			if (rotateValue >= 0.0)
			{
				rotateValue = (rotateValue * rotateValue);
			}
			else
			{
				rotateValue = -(rotateValue * rotateValue);
			}
		}

		if (moveValue > 0.0)
		{
			if (rotateValue > 0.0)
			{
				leftMotorOutput = moveValue - rotateValue;
				rightMotorOutput = max(moveValue, rotateValue);
			}
			else
			{
				leftMotorOutput = max(moveValue, -rotateValue);
				rightMotorOutput = moveValue + rotateValue;
			}
		}
		else
		{
			if (rotateValue > 0.0)
			{
				leftMotorOutput = - max(-moveValue, rotateValue);
				rightMotorOutput = moveValue + rotateValue;
			}
			else
			{
				leftMotorOutput = moveValue - rotateValue;
				rightMotorOutput = - max(-moveValue, -rotateValue);
			}
		}
		MotorOut(leftMotorOutput, rightMotorOutput);
	}
	
	void MotorOut(float leftOutput, float rightOutput)
	{
		if (LF != NULL)
		{
			LF->Set((Limit(leftOutput))*-1);
			LR->Set((Limit(leftOutput))*-1);
		}

		if (RF != NULL)
		{
			RF->Set(Limit(rightOutput));
			RR->Set(Limit(rightOutput));
		}

		//m_safetyHelper->Feed();
	}

	float Limit(float num)
	{
		if (num > 1.0)
		{
			return 1.0;
		}
		if (num < -1.0)
		{
			return -1.0;
		}
		return num;
	}
};

START_ROBOT_CLASS(RobotDemo);

