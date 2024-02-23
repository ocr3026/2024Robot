// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;	
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.keybinds.*;
import frc.robot.subsystems.*;
import frc.robot.keybinds.drivers.Tatum;
import frc.robot.keybinds.manipulators.Evan;
import edu.wpi.first.math.geometry.Pose2d;

public class RobotContainer {

	
	// Subsystems
	public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

	public ShootAuto shootAuto = new ShootAuto(shooterSubsystem);

	// Commands
	ClimberSubsystem climberSubsystem = new ClimberSubsystem();

	ServoCommand servoCommand = new ServoCommand(shooterSubsystem);
	AutoAim autoAim = new AutoAim(swerveSubsystem);


	public RobotCentric robotCentricCommand = new RobotCentric(swerveSubsystem);
	public FieldCentric fieldCentricCommand = new FieldCentric(swerveSubsystem);
	public Shoot shootCommand = new Shoot(shooterSubsystem);

	//public ClimbAtSpeed windUp = new ClimbAtSpeed(-.1, climberSubsystem);
	//public ClimbAtSpeed unWind = new ClimbAtSpeed(.1, climberSubsystem);


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

	// PathPlanner
	private SendableChooser<Command> autoChooser;

	public RobotContainer(DoubleSupplier getPeriodFn) {
		NamedCommands.registerCommand("Zero", new InstantCommand( () -> swerveSubsystem.resetPose(new Pose2d())));
		NamedCommands.registerCommand("Shoot", shootAuto);
		NamedCommands.registerCommand("Intake", new RunCommand(() -> shooterSubsystem.setIntakeVoltage(12)));
		NamedCommands.registerCommand("ZeroShoot", new RunCommand(() -> shooterSubsystem.setFlywheelSpeeds(0, 0)));
		NamedCommands.registerCommand("ZeroIntake", new RunCommand(() -> shooterSubsystem.setIntakeVoltage(0)));

		//justin's zone
		//FREE DIZZO
	
		//Path Planner (Autonomous Program) initialization
		AutoBuilder.configureHolonomic(() -> swerveSubsystem.getPose(),//where robot is
		 							(Pose2d pose) -> swerveSubsystem.resetPose(pose), //Tell Robot where it is
									() -> swerveSubsystem.speedGetter(), //How fast robot going
									(ChassisSpeeds speeds) -> swerveSubsystem.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false),   //Drive robot  
									new HolonomicPathFollowerConfig(
                    				new PIDConstants(1, 0.0, 0.0), // Translation PID constants
                    				new PIDConstants(1, 0.0, 0.0), // Rotation PID constants
                    	Constants.maxSpeed, // Max module speed, in m/s9
                    	Constants.frontLeftModulePos.getNorm(), // Drive base radius in meters. Distance from robot center to furthest module.
                    				new ReplanningConfig() // Default path replanning config. See the API for the options here
            ), () -> {  
			var alliance = DriverStation.getAlliance();
			if (alliance.isPresent()) {
			  return alliance.get() == DriverStation.Alliance.Red;
			}
			return false;}, swerveSubsystem);

			

		getPeriod = getPeriodFn;

		manipulatorChooser.setDefaultOption("Evan", evanProfile);
		driverChooser.setDefaultOption("Tatum", tatumProfile);

		manipulatorBinds = manipulatorChooser.getSelected();
		driverBinds = driverChooser.getSelected();

		configureCallbacks();
		configureBindings();

		//LOCK UP J HAUS

		//end of justins zone 
		
		// FREE J HAUS
		swerveSubsystem.setDefaultCommand(fieldCentricCommand);
		//Set your auto
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("AutoChooser", autoChooser);
	}

	private void configureBindings() {

		Constants.xbox.leftBumper().whileTrue(servoCommand);

		Constants.rotationJoystick.button(1).whileTrue(autoAim);


		manipulatorBinds.shootTrigger().whileTrue(shootCommand);

		manipulatorBinds.intakeTrigger().whileTrue(new InstantCommand(() -> {
			shooterSubsystem.setIntakeVoltage(9);
		})).whileFalse(new InstantCommand(() -> {
			shooterSubsystem.setIntakeVoltage(0);
		}));

		manipulatorBinds.exhaustTrigger().whileTrue(new InstantCommand(() -> {
			shooterSubsystem.setIntakeVoltage(-4);
		})).onFalse(new InstantCommand(() -> {
			shooterSubsystem.setIntakeVoltage(0);
		}));

		//manipulatorBinds.windUpTrigger().whileTrue(windUp);
		//manipulatorBinds.unwindTrigger().whileTrue(unWind);
		
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
		Constants.translationJoystick.button(12).whileTrue(new InstantCommand(() -> swerveSubsystem.resetPose(new Pose2d())));
		onEnableCallback.onTrue(new InstantCommand(() -> {
			manipulatorBinds = manipulatorChooser.getSelected();
			driverBinds = driverChooser.getSelected();
		}));
		
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
