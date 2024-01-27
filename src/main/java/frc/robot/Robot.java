// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;

	@Override
	public void robotInit() {
		m_robotContainer = new RobotContainer(this::getPeriod);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {
		// TODO: Remove tuna fish
		m_robotContainer.swerveSubsystem.frontLeftModule.setTuna();
		m_robotContainer.swerveSubsystem.frontRightModule.setTuna();
		m_robotContainer.swerveSubsystem.rearLeftModule.setTuna();
		m_robotContainer.swerveSubsystem.rearRightModule.setTuna();
	}

	@Override
	public void autonomousInit() {
		Constants.gyro.reset();
		Constants.gyro.setGyroAngle(Constants.gyro.getYawAxis(), Constants.initialYaw);

		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		Constants.gyro.reset();
		Constants.gyro.setGyroAngle(Constants.gyro.getYawAxis(), Constants.initialYaw);

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
