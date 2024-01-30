// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.keybinds.*;
import frc.robot.keybinds.drivers.Tatum;
import frc.robot.keybinds.manipulators.Evan;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
	// Subsystems
	public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

	// Commands
	public RobotCentric robotCentricCommand = new RobotCentric(swerveSubsystem);
	public FieldCentric fieldCentricCommand = new FieldCentric(swerveSubsystem);

	// Keybinds
	Evan evanProfile = new Evan();
	Tatum tatumProfile = new Tatum();

	ManipulatorProfile manipulatorBinds;
	SendableChooser<ManipulatorProfile> manipulatorChooser = new SendableChooser<>();
	DriverProfile driverBinds;
	SendableChooser<DriverProfile> driverChooser = new SendableChooser<>();

	// Callbacks
	public Trigger onEnableCallback = new Trigger(() -> { return DriverStation.isEnabled(); });

	// TimedRobot functions
	public static DoubleSupplier getPeriod;

	public RobotContainer(DoubleSupplier getPeriodFn) {
		Constants.gyro.reset();

		getPeriod = getPeriodFn;

		manipulatorChooser.setDefaultOption("Evan", evanProfile);
		driverChooser.setDefaultOption("Tatum", tatumProfile);

		manipulatorBinds = manipulatorChooser.getSelected();
		driverBinds = driverChooser.getSelected();

		configureCallbacks();
		configureBindings();

		swerveSubsystem.setDefaultCommand(fieldCentricCommand);
	}

	private void configureBindings() {
		if(Constants.tunaFish) {
			SmartDashboard.putNumber("driveKs", swerveSubsystem.frontLeftModule.driveFeedForward.ks); 
			SmartDashboard.putNumber("driveKv", swerveSubsystem.frontLeftModule.driveFeedForward.kv);
			SmartDashboard.putNumber("driveKa", swerveSubsystem.frontLeftModule.driveFeedForward.ka);

			SmartDashboard.putNumber("driveP", swerveSubsystem.frontLeftModule.drivePID.getP()); 
			SmartDashboard.putNumber("driveI", swerveSubsystem.frontLeftModule.drivePID.getI());
			SmartDashboard.putNumber("driveD", swerveSubsystem.frontLeftModule.drivePID.getD());

			SmartDashboard.putNumber("steerP", swerveSubsystem.frontLeftModule.steerPID.getP()); 
			SmartDashboard.putNumber("steerI", swerveSubsystem.frontLeftModule.steerPID.getI());
			SmartDashboard.putNumber("steerD", swerveSubsystem.frontLeftModule.steerPID.getD());
		}
	}

	private void configureCallbacks() {
		onEnableCallback.onTrue(new InstantCommand(() -> {
			manipulatorBinds = manipulatorChooser.getSelected();
			driverBinds = driverChooser.getSelected();
		}));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
