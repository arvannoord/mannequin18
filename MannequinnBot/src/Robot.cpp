/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include <time.h>
#include "Timer.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <WPILib.h>
#include "ctre/Phoenix.h"

#include "AHRS.h"

class Robot : public frc::IterativeRobot {
public:


	Compressor *grabComp = new Compressor(0);
	DoubleSolenoid *gSol1 = new DoubleSolenoid(0, 1);
	WPI_TalonSRX *lf = new WPI_TalonSRX(0); //left front
	WPI_TalonSRX *rf = new WPI_TalonSRX(1); //right front
	WPI_TalonSRX *lb = new WPI_TalonSRX(2); //left rear
	WPI_TalonSRX *rb = new WPI_TalonSRX(3); //right rear
	WPI_TalonSRX *arm = new WPI_TalonSRX(7); //pickup wheels left


	Ultrasonic *ultra = new Ultrasonic(0, 1); //ultrasonic
	double distance = 0;
	double driveSpeed = 0;

	AHRS *gyro;

	Joystick *controller;
			MecanumDrive *robotDrive;

			//joystick
			int joystickX = 0;
			int joystickY = 1;
			int joystickrot = 2;

			//controller buttons

			int turboButton = 5; //do something
			int armSwingf = 8; //swings the arm
			int armSwingb = 6; //swings arm other way
			int pneumatic = 1; //whatever the pneumatic does
			int pneumatic2 = 4; //other direction




	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


		controller = new Joystick(0);
		robotDrive = new MecanumDrive(*lf, *lb, *rf, *rb);
		robotDrive->SetExpiration(0.5);
		robotDrive->SetSafetyEnabled(false);

		gyro = new AHRS(SPI::Port::kMXP);
		gyro->ZeroYaw();




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



		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void MecDrive(double xAxis, double yAxis, double rot) //homemade mecanum drive!
		{
			double noMove = 0.2; //Dead area of the axes
			double maxSpeed = .5; //normal speed (not turbo)

			if (fabs(xAxis) < noMove)
				xAxis = 0.0;

			if (fabs(yAxis) < noMove)
				yAxis = 0.0;

			if (fabs(rot) < noMove)
				rot = 0.0;

			if (Joystick->GetRawButton(turboButton)) //fix this
				maxSpeed = 1;

			else
				maxSpeed = .5;

			double lfSpeed = -yAxis - xAxis - rot;
			double rfSpeed = +yAxis - xAxis - rot;
			double rrSpeed = +yAxis + xAxis - rot;
			double lrSpeed = -yAxis + xAxis - rot;

			if (fabs(lfSpeed) > 1)
				lfSpeed = fabs(lfSpeed) / lfSpeed;

			if (fabs(lrSpeed) > 1)
				lrSpeed = fabs(lrSpeed) / lrSpeed;

			if (fabs(rfSpeed) > 1)
				rfSpeed = fabs(rfSpeed) / rfSpeed;

			if (fabs(rrSpeed) > 1)
				rrSpeed = fabs(rrSpeed) / rrSpeed;

			lf -> Set(ControlMode::PercentOutput, lfSpeed*maxSpeed);
			lb -> Set(ControlMode::PercentOutput, lrSpeed*maxSpeed);
			rf -> Set(ControlMode::PercentOutput, rfSpeed*maxSpeed);
			rb -> Set(ControlMode::PercentOutput, rrSpeed*maxSpeed);
		}

	void TeleopInit() {}

	void TeleopPeriodic() {

		double joyY=Joystick->GetRawAxis(joystickY);
		double joyX=Joystick->GetRawAxis(joystickX);
		double joyRot=Joystick->GetRawAxis(joystickrot);
		MecDrive(joyX,-joyY,joyRot);
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
