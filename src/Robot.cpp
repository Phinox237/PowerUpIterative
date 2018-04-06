/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <WPILib.h>
#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <Spark.h>
#include <Talon.h>
#include <Timer.h>
#include <ADXRS450_Gyro.h>
#include <SPI.h>
#include <DigitalInput.h>
#include <CameraServer.h>

using namespace std;
using namespace frc;

double g_Angle;

class Robot : public IterativeRobot {
public:

	void Zero() {
		RobotDrive.ArcadeDrive(0,0);
	}

	void ShootBall(double speed) {
		clawL.Set(-speed);
		clawR.Set(speed);
		Wait(0.5);
		clawL.Set(0);
		clawR.Set(0);
	}

	void DropClaw() {
		clawY.Set(0.85);
		Wait(1.3);
		clawY.Set(0);
		clawY.Set(-0.1);
		Wait(0.15);
		clawY.Set(0);
	}

	void CrossLine() {
		RobotDrive.ArcadeDrive(0.75,0.1);
		Wait (2);
		Zero();

	}
	void ToScaleL() {
		RobotDrive.ArcadeDrive(0.75, 0.15);
		Wait(1.9);
		Zero();
		clawY.Set(0.85);
		Wait(3.8);
		clawY.Set(0);
	}
	void DoScaleL() {
		CrossLine();
		ToScaleL();
		RobotDrive.ArcadeDrive(0, 0.8);
		Wait(0.9);
		Zero();
		RobotDrive.ArcadeDrive(0.6, 0);
		Wait(1.3);
		Zero();
		ShootBall(0.3);
		Wait(0.3);
		RobotDrive.ArcadeDrive(-0.75, 0);
		Wait(0.3);
		Zero();
	}
	void DoSwitchL() {
		RobotDrive.ArcadeDrive(0.75,0.1);
		Wait (2.25);
		RobotDrive.ArcadeDrive(0,0);
		Wait(0.1);

		RobotDrive.ArcadeDrive(0, 0.75);
		Wait(1.5);
		Zero();
		Wait(0.1);

		clawY.Set(0.85);
		Wait(1.4);
		clawY.Set(0);
		Wait(0.1);

		RobotDrive.ArcadeDrive(0.65, 0);
		Wait(0.4);
		Zero();

		ShootBall(0.4);
	}
	void ToScaleR() {
		RobotDrive.ArcadeDrive(0.75, 0.15);
		Wait(1.9);
		Zero();
		clawY.Set(0.85);
		Wait(3.8);
		clawY.Set(0);
	}
	void DoScaleR() {
		CrossLine();
		ToScaleR();
		RobotDrive.ArcadeDrive(0, -0.8);
		Wait(0.9);
		Zero();
		RobotDrive.ArcadeDrive(0.6, 0);
		Wait(1.3);
		Zero();
		ShootBall(0.3);
		Wait(0.3);
		RobotDrive.ArcadeDrive(-0.75, 0);
		Wait(0.3);
		Zero();
	}
	void DoSwitchR() {
		RobotDrive.ArcadeDrive(0.75,0.1);
		Wait (2.25);
		RobotDrive.ArcadeDrive(0,0);
		Wait(0.1);

		RobotDrive.ArcadeDrive(0, -0.75);
		Wait(1.5);
		Zero();
		Wait(0.1);

		clawY.Set(0.85);
		Wait(1.4);
		clawY.Set(0);
		Wait(0.1);

		RobotDrive.ArcadeDrive(0.65, 0);
		Wait(0.4);
		Zero();

		ShootBall(0.4);
	}

    static void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(640, 480);
        cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
        cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
    }

	void RobotInit() {

		c_Mode.AddDefault(autoOnLeft, autoOnLeft);
		c_Mode.AddObject(autoOnRight, autoOnRight);
		c_Mode.AddObject(autoOnCenter, autoOnCenter);
		c_Mode.AddObject(autoIdle, autoIdle);
		c_Prioritize.AddDefault(DoSwitch, DoSwitch);
		c_Prioritize.AddObject(DoScale, DoScale);
		c_Prioritize.AddObject(CrossLineOnly, CrossLineOnly);


		SmartDashboard::PutData("Auto Modes", &c_Mode);
		SmartDashboard::PutData("Priority", &c_Prioritize);

		gyro.Calibrate();
		/*std::thread visionThread(VisionThread);
		visionThread.detach(); */
		gyro.Reset();
		d_setLeft.SetInverted(false);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		string autoSelected = c_Mode.GetSelected();
		string prioritizeSelected = c_Prioritize.GetSelected();
		// string autoSelected = SmartDashboard::GetString(
		// "Auto Selector", kAutoNameDefault);
		cout << "Auto selected: " << autoSelected << endl;
		cout << "Prioritize: " << prioritizeSelected << endl;

		// MotorSafety improves safety when motors are updated in loops
		// but is disabled here because motor updates are not looped in
		// this autonomous mode.
		d_setLeft.SetInverted(false);
		clawL.SetInverted(true);
		RobotDrive.SetSafetyEnabled(false);

		string gameData = DriverStation::GetInstance().GetGameSpecificMessage();
//The block below is for assigning priorities to a numerical value...
		int priorityInt;
		if(prioritizeSelected == DoSwitch){
			priorityInt = 0;
		}
		else if(prioritizeSelected == DoScale){
			priorityInt = 1;
		}
		else if(prioritizeSelected == CrossLineOnly){
			priorityInt = 2;
		} else {
			cout << "No Priority Detected." << endl;
		}
//END BLOCK

//The block below is for assigning game data entries to a numerical value...
		//Switch
		int switchSideInt;
		if(gameData[0] == 'L'){
			switchSideInt = 0;
			cout << "Switch on Left" << endl;
		}
		else if(gameData[0] == 'R'){
			switchSideInt = 1;
			cout << "Switch on Right" << endl;
		} else {
			cout << "No SwitchSide Detected." << endl;
		}

		//Scale
		int scaleSideInt;
		if(gameData[1] == 'L'){
			scaleSideInt = 0;
		}
		else if(gameData[1] == 'R'){
			scaleSideInt = 1;
		} else {
			cout << "No ScaleSide Detected." << endl;
		}
//END BLOCK
		//Dropping Claw
/*		clawY.Set(1);
		Wait(0.5);
		clawY.Set(0);
		Wait(0.05);
		clawY.Set(-1);
		Wait(0.45);
		clawY.Set(0); */

		gyro.Reset();
		if (autoSelected == autoOnCenter) {
			cout << "Running Center Auton..." << endl;
			switch(priorityInt){
			case 0:
				switch (switchSideInt){
				case 0:
					cout << "Going for Left Switch" << endl;

					RobotDrive.ArcadeDrive(0.75,0);
					Wait(0.75);
					Zero();
					Wait(0.1);

					RobotDrive.ArcadeDrive(0, -0.8);
					Wait(0.85);
					Zero();
					Wait(0.1);

					RobotDrive.ArcadeDrive(0.75,0);
					Wait(1.75);
					Zero();
					Wait(0.1);

					RobotDrive.ArcadeDrive(0, 0.8);
					Wait(0.7);
					Zero();
					Wait(0.1);

					clawY.Set(0.85);
					Wait(1.3);
					clawY.Set(0);

					RobotDrive.ArcadeDrive(0.75, 0);
					Wait(1);
					Zero();
					Wait(0.25);

					ShootBall(0.4);
					Wait(0.25);

					clawY.Set(-0.1);
					Wait(0.15);
					clawY.Set(0);
					break;

				case 1/*Right*/:
					//In front of the center station and going to right side of switch.
					cout << "Going for Right Switch" << endl;

					clawY.Set(0.85);
					Wait(1.4);
					clawY.Set(0);
					Wait(0.1);
					RobotDrive.ArcadeDrive(0.75,0.2);
					Wait(2);
					Zero();
					ShootBall(0.5);
					break;

				default: //NO CODE ERROR
					cout << "No DS Data, Crossing Line" << endl;
					CrossLine();
					break;
				}
				break;

				default:
					CrossLine();
					break;
			}
		}

		else if (autoSelected == autoOnRight) {
			cout << "Running Right Auton..." << endl;

			gyro.Reset();

			switch(priorityInt) {
				case 0:
					switch(switchSideInt) {
					case 0:
						switch(scaleSideInt) {
						case 0:
							CrossLine();
							break;
						case 1:
							DoScaleR();
							break;
						default:
							CrossLine();
							break;
						}
						break;
					case 1 :
						DoSwitchR();
						break;
					}
					break;
				default:
					CrossLine();
					break;
			}
		}

		else if (autoSelected == autoOnLeft) {
			cout << "Running Left Auton..." << endl;

			switch(priorityInt) {
			case 0:
				switch(switchSideInt) {
				case 0:
					DoSwitchL();
					break;
				case 1:
					switch(scaleSideInt) {
					case 0:
						DoScaleL();
						break;
					case 1:
						CrossLine();
						break;
					}
					break;
				default:
					CrossLine();
					break;
				}
			break;
			case 1:
				switch (scaleSideInt) {
				case 0:
					DoScaleL();
					break;
				case 1:
					switch (switchSideInt) {
					case 0:
						DoSwitchL();
						break;
					case 1:
						CrossLine();
						break;
					}
					break;
				}
				break;
			default:
				CrossLine();
				break;
			}
		}
		else if (autoSelected == autoIdle) {
			cout << "Doing /Literally/ Nothing in Auton..." << endl;
			gyro.Reset();

		}

	}

	void AutonomousPeriodic() {
		g_Angle = gyro.GetAngle();
	}

	void TeleopInit() {

		d_setLeft.SetInverted(false);
		clawL.SetInverted(true);
		RobotDrive.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {
			// Drive
			RobotDrive.ArcadeDrive(-stickDrive.GetY(), stickDrive.GetZ());
			SmartDashboard::PutNumber("Gyro Angle", gyro.GetAngle());

			// Claw up/down
			if (stickAux.GetRawButton(3)) {
				clawY.Set(0.8);
			} else if (stickAux.GetRawButton(4)) {
				clawY.Set(-0.8);
			} else if (stickAux.GetRawButton(5)) {
				clawY.Set(1);
			} else if (stickAux.GetRawButton(6)) {
				clawY.Set(-1);
			} else {
				clawY.Set(0);
			}

			//Cube-related code
			if (stickAux.GetRawButton(2)) {
				clawL.Set(1);
				clawR.Set(-1);

			} else if (stickAux.GetRawButton(1)) {
				clawL.Set(-0.65);
				clawR.Set(0.65);

			} else if (stickAux.GetY() > 0.2 || stickAux.GetY() < -0.2){
				clawL.Set(stickAux.GetY()*0.5);
				clawR.Set(-stickAux.GetY()*0.5);

			} else if (stickAux.GetZ() > 0.2 || stickAux.GetZ() < -0.2){
				clawL.Set(stickAux.GetZ()*0.25);
				clawR.Set(stickAux.GetZ()*0.25);

			} else {
				clawL.Set(0);
				clawR.Set(0);
			}

			//Climber
			if (stickAux.GetRawButton(11)) {
				mClimb.Set(0.8);
			} else if (stickAux.GetRawButton(12)) {
				mClimb.Set(-0.8);
			} else if (stickAux.GetRawButton(10)) {
				mClimb.Set(0.5);
			} else {
				mClimb.Set(0);
			}

			// The motors will be updated every 5ms
			Wait(0.005);
		}
	}

	void TeleopPeriodic() {
		g_Angle = gyro.GetAngle();
	}

	void TestPeriodic() {}

private:
	//For switch/scale state
	string gameData;

	//Left Drive
	Spark driveFL{0};
	Spark driveRL{1};
	SpeedControllerGroup d_setLeft{driveRL, driveFL};

	//Right Drive
	Spark driveFR{2};
	Spark driveRR{3};
	SpeedControllerGroup d_setRight{driveRR, driveFR};

	DifferentialDrive RobotDrive{d_setLeft, d_setRight};

	//Claw motors
	Victor clawL{4};
	Victor clawR{5};
	Spark clawY{6};

	//Climing
	Spark mClimb{7};

	//Joysticks
	Joystick stickDrive{0};
	Joystick stickAux{1};

	//Gyro
	ADXRS450_Gyro gyro{SPI::Port::kOnboardCS0};

	//Servo
	Servo servo{7};

	//Limit Switches
	DigitalInput limit0{0};
	DigitalInput limit1{1};
	DigitalInput limit2{2};
	DigitalInput limit3{3};

	//Auton stuff
	SendableChooser<string> c_Prioritize;
	const string DoScale = "Scale";
	const string DoSwitch = "Switch";
	const string CrossLineOnly = "Cross Line Only";

	SendableChooser<string> c_Mode;
	const string autoIdle = "DoNothing";
	const string autoOnLeft = "StartInLeft";
	const string autoOnRight = "StartInRight";
	const string autoOnCenter = "StartInCenter";

};

START_ROBOT_CLASS(Robot)
