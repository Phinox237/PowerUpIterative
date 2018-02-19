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

using namespace std;
using namespace frc;

double g_Angle;

class Robot : public IterativeRobot {
public:

	void ShootBall() {
		clawL.Set(-1);
		clawR.Set(1);
		Wait(0.2);
		clawL.Set(0);
		clawR.Set(0);
	}

	void DropClaw() {
		RobotDrive.ArcadeDrive(1,0);
		Wait(0.25);
		RobotDrive.ArcadeDrive(-1,0);
		Wait(0.25);
		RobotDrive.ArcadeDrive(0,0);
	}

	void CrossLine() {
		DropClaw();
		RobotDrive.ArcadeDrive(0.75,-gyro.GetAngle()*0.003);
		Wait (2);
		RobotDrive.ArcadeDrive(0,0);

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
		SmartDashboard::PutData("Gyro Angle", &g_Angle);

		gyro.Calibrate();
		gyro.Reset();
		d_setLeft.SetInverted(true);
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
		// string autoSelected = SmartDashboard::GetString(
		// "Auto Selector", kAutoNameDefault);
		cout << "Auto selected: " << autoSelected << endl;

		string prioritizeSelected = c_Prioritize.GetSelected();
		cout << "Prioritize: " << prioritizeSelected << endl;

		// MotorSafety improves safety when motors are updated in loops
		// but is disabled here because motor updates are not looped in
		// this autonomous mode.
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
		}
//END BLOCK

//The block below is for assigning game data entries to a numerical value...
		//Switch
		int switchSideInt;
		if(gameData[0] == 'L'){
			switchSideInt = 0;
		}
		else if(gameData[0] == 'R'){
			switchSideInt = 1;
		}

		//Scale
		int scaleSideInt;
		if(gameData[1] == 'L'){
			scaleSideInt = 0;
		}
		else if(gameData[1] == 'R'){
			scaleSideInt = 1;
		}
//END BLOCK

		if (autoSelected == autoOnCenter) {
			// Custom Auto goes here
			cout << "Running Center Auton..." << endl;

			switch (switchSideInt){
			case 0/*Left*/:
				//In front of the center station and going to left side of switch.
				DropClaw();
				RobotDrive.ArcadeDrive(0.5,0);
				Wait(0.5);
				do {
					RobotDrive.ArcadeDrive(0,0.5);
				} while(gyro.GetAngle() > -45);
				do {
					RobotDrive.ArcadeDrive(0.5,-0.25);
				} while(gyro.GetAngle() > 0);

								break;

				break;
			case 1/*Right*/:
				//In front of the center station and going to right side of switch.
				DropClaw();
				RobotDrive.ArcadeDrive(0.5,0);
				Wait(0.5);
				do {
					RobotDrive.ArcadeDrive(0,0.5);
				} while(gyro.GetAngle() < 45);
				do {
				RobotDrive.ArcadeDrive(0.5,-0.25);
				} while(gyro.GetAngle() > 0);

				break;
			default/*NO CODE ERROR*/:
				cout << "NO CODE GIVEN... DRIVING PAST AUTO LINE" << endl;
				RobotDrive.ArcadeDrive(0.5, -gyro.GetAngle()*0.003);

				break;
			}


		}
		else if (autoSelected == autoOnRight) {
			cout << "Running Right Auton..." << endl;

			gyro.Reset();

			switch(priorityInt) {

			case 1 :
				//Prioritize scale
				switch(scaleSideInt) {
				case 0 :
					//Scale is on left, checking switch...
					switch(switchSideInt) {
					case 1 :
						//Do switch
						DropClaw();
						RobotDrive.ArcadeDrive(-1, -gyro.GetAngle()*0.003);
						Wait(0.8);
						do {
						RobotDrive.ArcadeDrive(0, -0.33);
						} while (gyro.GetAngle() < 85);
						RobotDrive.ArcadeDrive(0.5, -gyro.GetAngle()*0.003);
						Wait(0.3);
						clawY.Set(0.75);
						Wait(0.3);
						ShootBall();
						break;
					default :
						CrossLine();
						break;

					}
					break;
				case 1 :
					//Scale is on right, doing scale...
					DropClaw();
					RobotDrive.ArcadeDrive(0.75, -gyro.GetAngle()*0.003);
					Wait(2);
					do {
						RobotDrive.ArcadeDrive(0, 0.33);
					} while (gyro.GetAngle() > 1);
					RobotDrive.ArcadeDrive(0,0);
					clawY.Set(0.7);
					Wait(1.3);
					clawY.Set(0);
					break;

				}
				break;

			case 0 :
				//Prioritize switch
				switch(switchSideInt) {
				case 0 :
					//Switch is on left, checking scale...
					switch(scaleSideInt) {
					case 1 :
						//Scale is on right, doing scale
						DropClaw();
						RobotDrive.ArcadeDrive(0.75,-gyro.GetAngle()*0.003);
						Wait(1);
						do {
						RobotDrive.ArcadeDrive(0,-0.33);
						} while (gyro.GetAngle() > -85);
						break;

					default :
						CrossLine();
						break;

					}
					break;

				case 1 :
					//Switch is on right, doing switch
					DropClaw();
					RobotDrive.ArcadeDrive(0.75,-gyro.GetAngle()*0.003);
					do {
						RobotDrive.ArcadeDrive(0,-0.33);
					} while (gyro.GetAngle() > -85);

					break;
				}
				break;

			case 2 :
				CrossLine();
				break;

			}

		}
		else if (autoSelected == autoOnLeft) {
			cout << "Running Left Auton..." << endl;

			gyro.Reset();

			switch(priorityInt){

			case 1 :
				switch(scaleSideInt){
				case 0 :
					//Do scale
					break;
				case 1 :
					switch(switchSideInt){
					case 0 :
						//Do switch
						break;
					default :
						CrossLine();
						break;
					}
					break;
				}
				break;
			case 0 :
				switch(switchSideInt){
				case 0 :
						//Do switch
						break;
				case 1 :
					switch(scaleSideInt){
					case 0 :
						//Do scale
						break;
					default :
						CrossLine();
						break;

						}
					break;
				}
				break;
			case 2 :
				//Cross line
				break;
			}
		}
		else if (autoSelected == autoIdle) {
			cout << "Doing Nothing in Auton..." << endl;
			gyro.Reset();

		}

	}

	void AutonomousPeriodic() {
		g_Angle = gyro.GetAngle();
	}

	void TeleopInit() {
		RobotDrive.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled()) {

			// Drive
			RobotDrive.TankDrive(stickDriveL.GetY(), -stickDriveR.GetY());

			// Claw up/down
			if (stickAux.GetRawButton(3)) {
				clawY.Set(0.7);
			} else if (stickAux.GetRawButton(4)) {
				clawY.Set(-0.7);
			} else {
				clawY.Set(0);
			}

			//Cube-related code
			if (stickAux.GetRawButton(2)) {
				clawL.Set(1);
				clawR.Set(-1);

			} else if (stickAux.GetRawButton(1)) {
				clawL.Set(-0.8);
				clawR.Set(0.8);

			} else if (stickAux.GetY() > 0.2 || stickAux.GetY() < -0.2) {
				clawL.Set(stickAux.GetY()*0.25);
				clawR.Set(-stickAux.GetY()*0.25);

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
			} else {
				mClimb.Set(0);
			}

			// The motors will be updated every 5ms
			Wait(0.005);
		}
	}

	void TeleopPeriodic() {
		g_Angle = gyro.GetAngle();
		SmartDashboard::PutData(g_Angle, g_Angle);
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
	Joystick stickDriveL{0};
	Joystick stickDriveR{1};
	Joystick stickAux{2};

	//Gyro
	ADXRS450_Gyro gyro{SPI::Port::kOnboardCS0};

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

	SendableData<double> g_Angle;
};

START_ROBOT_CLASS(Robot)
