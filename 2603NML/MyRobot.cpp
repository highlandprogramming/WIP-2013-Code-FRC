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
//#include "SmartDashboard/SendableChooser.h"
//#include "SmartDashboard/Sendable.h"
//#include "SmartDashboard/NamedSendable.h"
//#include "Target.h"
//#include "BinaryImage.h"
//#include "VisionAPI.h"
// #@$%#@$^%^@$%^!#$%#$^@$%&&$%$^!$#%#@$% http://www.chiefdelphi.com/forums/showthread.php?t=101624 is the forum page
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
	DigitalInput *LimitSwitch;
	int intAutoCtrl;
	int intDelay;
	int intCount;
	int intSecondWait;
	bool blnLowTime;
	bool blnShooterSpd;
	bool blnReverse;
	float fltStick1X;
	float fltStick1Y;
	Timer timerLowHang;
	Timer timerShift;
	Timer timerFire;
	Timer timerShooter;
	Timer timerDriveCtrl;
	Timer timerCamera;
	Timer timerReverse;




public:
	RobotDemo(void):
		myRobot(1,3,2,4),
		myShooter1(5),
		myShooter2(6)
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
		LimitSwitch = new DigitalInput(3);
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
		GetWatchdog().SetEnabled(true);
		GetWatchdog().SetExpiration(1);
		GetWatchdog().Feed();
		s[0]->Set(true);
		s[1]->Set(true);
		SmartDashboard::PutString("Gear","High");
		compressor->Start();
		myRobot.SetSafetyEnabled(true);


		int WaitDash = 0;
		int FireDash = 0;
		int intPause = 0;
		GetWatchdog().Feed();

		// Incase we need to recreate the controls for Autonomous.
		//SmartDashboard::PutNumber("fire_wait", 3);
		//SmartDashboard::PutNumber("fire_amount", 3);
		//SmartDashboard::PutNumber("fire_pause", 5);

		WaitDash = static_cast<int>(SmartDashboard::GetNumber("fire_wait"));
		FireDash = static_cast<int>(SmartDashboard::GetNumber("fire_amount"));
		intPause = static_cast<int>(SmartDashboard::GetNumber("fire_pause"));
		GetWatchdog().Feed();


		/* CAMERA THRESHOLD AND DECLARATIONS, UNCOMMENT IF CAMERA READDED TO AUTONOMOUS
		//Their threshold values suck DDDD, from the NIvision assistant will be below.
		//Threshold threshold(60, 100, 90, 255, 20, 255);	//HSV threshold criteria, ranges are in that order ie. Hue is 60-100
		//Threshold threshold(100, 255, 230, 255, 140, 255); //New threshold values
		Threshold threshold(0, 255, 0, 255, 221, 255);
		
		ParticleFilterCriteria2 criteria[] = {
				{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}
		};												//Particle filter criteria, used to filter out small particles
		// AxisCamera &camera = AxisCamera::GetInstance();	//To use the Axis camera uncomment this line
				
		*/
		while (IsAutonomous() && IsEnabled())
		{

			myShooter1.Set(-1);
			myShooter2.Set(-1);
			GetWatchdog().Feed();


			GetWatchdog().Feed();

			// Wait before firing.
			for(intAutoCtrl = 0; intAutoCtrl < (1);intAutoCtrl++)
			{
				GetWatchdog().Feed();
				for(intDelay = 0; intDelay< (((intPause)*2)+1);intDelay++)
				{
					GetWatchdog().Feed();
					Wait(0.5);
				}
				GetWatchdog().Feed();


				// Fire a number of frisbees as set in the dashboard.
				//for(intCount = 0; intCount < (FireDash+1); intCount++)
			//	{
					GetWatchdog().Feed();
					s[2]->Set(true);

					// Delay between firing each frisbees.
					for(int wait = 0; wait<((WaitDash)*2);wait++)
					{
						GetWatchdog().Feed();
						Wait(0.5);
					}
					s[2]->Set(false);
					GetWatchdog().Feed();
					for(intSecondWait = 0;intSecondWait < 3;intSecondWait++)
					{
						GetWatchdog().Feed();
						Wait(0.5);
					}
					GetWatchdog().Feed();
				}
				GetWatchdog().Feed();
			}

			myShooter1.Set(0);
			myShooter2.Set(0);
			GetWatchdog().Feed();
		//}

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
		bool blnReverse = false;

		float fltShoot;
		float fltSpeed = 1;

		int intFail = 0;
		
		timerLowHang.Reset();
		timerShift.Reset();
		timerFire.Reset();
		timerShooter.Reset();
		timerDriveCtrl.Reset();
		timerCamera.Reset();
		timerReverse.Reset();

		timerLowHang.Start();
		timerShift.Start();
		timerFire.Start();
		timerShooter.Start();
		timerDriveCtrl.Start();
		timerCamera.Start();
		timerReverse.Start();
		
		GetWatchdog().Feed();

		//sd->sendIOPortData();

		// Local variables.
		//float fltStick1X, fltStick1Y;

		while (IsOperatorControl())
		{
			if(timerReverse.Get() > 0.5)
			{
				if(stick1->GetRawButton(8) && blnReverse == false )
				{
					blnReverse = true;
					GetWatchdog().Feed();
					timerReverse.Reset();
					timerReverse.Start();
				}
				else if(stick1->GetRawButton(9) && blnReverse == true)
				{
					blnReverse = false;
					GetWatchdog().Feed();
					timerReverse.Reset();
					timerReverse.Start();
				}
			}
			if(blnReverse == false)
			{
				fltStick1Y = stick1->GetY();
				fltStick1X = stick1->GetX();
				SmartDashboard::PutBoolean("Reverse",false);
				GetWatchdog().Feed();
			}
			else if(blnReverse == true)
			{
				fltStick1Y = ((stick1->GetY())*(-1));
				fltStick1X = ((stick1->GetX())*(1));
				SmartDashboard::PutBoolean("Reverse",true);
				GetWatchdog().Feed();
			}
			myRobot.ArcadeDrive(fltStick1Y,fltStick1X);
			//myRobot.ArcadeDrive(stick1);
			GetWatchdog().Feed(); // Feed hungary demonic Watchdog.


			SmartDashboard::PutBoolean("Touching Tower?",LimitSwitch->Get());
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
			if(timerShift.Get() > 0.2)
			{
				if(stick1->GetRawButton(7) || stick1->GetTrigger())
				{
					if(blnShift == false)
					{
						GetWatchdog().Feed();
						s[0]->Set(false);
						s[1]->Set(true);
						SmartDashboard::PutString("Gear","Low");
						blnShift = true;
						timerShift.Stop();
						timerShift.Reset();
						timerShift.Start();
						GetWatchdog().Feed();
					}	
					else if (blnShift == true)
					{
						GetWatchdog().Feed();
						s[0]->Set(true);
						s[1]->Set(false);
						SmartDashboard::PutString("Gear","High");
						blnShift = false;
						timerShift.Stop();
						timerShift.Reset();
						timerShift.Start();
						GetWatchdog().Feed();
					}
				}		
			}
			if(stick1->GetRawButton(2) && blnLowHang == false && timerLowHang.Get() > 0.5)
			{
				blnLowTime = true;
				blnLowHang = true;
				timerLowHang.Stop();
				timerLowHang.Reset();
				timerLowHang.Start();
				GetWatchdog().Feed();
			}
			else if(stick1->GetRawButton(2) && blnLowHang == true && timerLowHang.Get() > 0.5)
			{
				blnLowTime = false;
				blnLowHang = false;
				timerLowHang.Stop();
				timerLowHang.Reset();
				timerLowHang.Start();
				GetWatchdog().Feed();
			}
			if(blnLowTime == true)
			{
				s[3]->Set(true);
				GetWatchdog().Feed();
			}
			else if (blnLowTime == false)
			{
				s[3]->Set(false);
				GetWatchdog().Feed();
			}

			if(stick1->GetRawButton(3))
			{

				mtdCameraCode();
				GetWatchdog().Feed();

			}

			if(stick2->GetTrigger() && intFail == 0 && timerFire.Get() > 0.8)
			{
				s[2]->Set(true);
				SmartDashboard::PutString("Shooter Piston","In");
				intFail = 1;
				GetWatchdog().Feed();
				timerFire.Stop();
				timerFire.Reset();
				timerFire.Start();
				GetWatchdog().Feed();
			}
			else if(stick2->GetTrigger() && intFail == 1 && timerFire.Get() > 0.8)
			{
				s[2]->Set(false);
				intFail = 0;
				SmartDashboard::PutString("Shooter Piston","Out");
				GetWatchdog().Feed();
				timerFire.Stop();
				timerFire.Reset();
				timerFire.Start();
				GetWatchdog().Feed();
			}

			if(stick2->GetRawButton(2) && blnShooterSpd == false && timerShooter.Get() > 0.5)
			{
				GetWatchdog().Feed();
				myShooter1.Set(-fltSpeed);
				myShooter2.Set(-fltSpeed);
				SmartDashboard::PutString("Shooter","On");
				SmartDashboard::PutNumber("Shooter Speed (%)",(fltSpeed)*(100));
				blnShooterSpd = true;
				GetWatchdog().Feed();
				timerShooter.Stop();
				timerShooter.Reset();
				timerShooter.Start();
				GetWatchdog().Feed();
			}
			else if(stick2->GetRawButton(2) && blnShooterSpd == true && timerShooter.Get() > 0.5)
			{
				GetWatchdog().Feed();
				myShooter1.Set(0);
				myShooter2.Set(0);
				SmartDashboard::PutString("Shooter","Off");
				SmartDashboard::PutNumber("Shooter Speed (%)",0);
				blnShooterSpd = false;
				GetWatchdog().Feed();
				//Wait(0.2);
				timerShooter.Stop();
				timerShooter.Reset();
				timerShooter.Start();
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
			Wait(0.1);
		}
	}

	void mtdCameraCode(void)
	{
		Threshold threshold(0, 255, 0, 255, 221, 255);

		ParticleFilterCriteria2 criteria[] = {{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}};		

		AxisCamera &camera = AxisCamera::GetInstance("10.26.3.11");
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(20);
		camera.WriteBrightness(50);


		//SmartDashboard::PutNumber("Test", 3);
		if(timerCamera.Get() > 0.1)
		{	
			ColorImage *image;
			//image = new RGBImage("/HybridLine_DoubleGreenBK3.jpg");		// get the sample image from the cRIO flash
			image = camera.GetImage();
			//camera.GetImage(image);				//To get the images from the camera comment the line above and uncomment this one
			//Wait(.1);
	
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
		timerCamera.Reset();
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
};

START_ROBOT_CLASS(RobotDemo)
