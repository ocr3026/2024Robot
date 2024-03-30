// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

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
import frc.robot.subsystems.SwerveSubsystem.DriveOrigin;
import frc.robot.keybinds.drivers.Tatum;
import frc.robot.keybinds.manipulators.Evan;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotContainer {

	
	// Subsystems
	public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
	public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

	public ShootAuto shootAuto = new ShootAuto(shooterSubsystem);
	public ClimberSubsystem climberSubsystem = new ClimberSubsystem();


	// Commands


	CamCommand camCommand = new CamCommand(shooterSubsystem);
	AutoAim autoAim = new AutoAim(swerveSubsystem, shooterSubsystem);
	ClimbAtSpeed climbAtSpeed = new ClimbAtSpeed(climberSubsystem);
	AutoAimAndShoot autoAimAndShoot = new AutoAimAndShoot(swerveSubsystem, shooterSubsystem);
	ClimbBalance climbBalance = new ClimbBalance(climberSubsystem, swerveSubsystem);
	DriveTo driveToRedSource = new DriveTo(swerveSubsystem, new Pose2d((new Translation2d(0.46, 0.62)), (new Rotation2d(130))));
	AutoAimInAuto autoAimInAuto = new AutoAimInAuto(shooterSubsystem);
	ZeroYaw zeroYaw = new ZeroYaw(swerveSubsystem);
	CamCommandAuto camCommandAuto = new CamCommandAuto(shooterSubsystem, 0.8);

	IntakeAuto intakeAuto = new IntakeAuto(shooterSubsystem);

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
		SmartDashboard.putNumber("SetCamPos", 0.5);
		SmartDashboard.putNumber("servoSet", 0);
		NamedCommands.registerCommand("Zero", new InstantCommand( () -> swerveSubsystem.resetPose(new Pose2d())));
		NamedCommands.registerCommand("Shoot", shootAuto);
		NamedCommands.registerCommand("Intake", intakeAuto);
		NamedCommands.registerCommand("IntakeWhileDrive", new InstantCommand(() -> shooterSubsystem.setIntakeVoltage(10)));
		NamedCommands.registerCommand("ZeroShoot", new RunCommand(() -> shooterSubsystem.setFlywheelVoltage(0, 0)));
		NamedCommands.registerCommand("ZeroIntake", new RunCommand(() -> shooterSubsystem.setIntakeVoltage(0)));
		NamedCommands.registerCommand("autoAim", autoAim);
		NamedCommands.registerCommand("LowerCam", camCommandAuto);
		NamedCommands.registerCommand("zeroYaw", zeroYaw);
		

		//justin's zone
		//FREE DIZZO
	
		//Path Planner (Autonomous Program) initialization
		AutoBuilder.configureHolonomic(() -> swerveSubsystem.autoGetPose(),//where robot is
		 							(Pose2d pose) -> swerveSubsystem.autoResetPose(pose), //Tell Robot where it is
									() -> swerveSubsystem.speedGetter(), //How fast robot going
									(ChassisSpeeds speeds) -> swerveSubsystem.drive(speeds.vxMetersPerSecond,-speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond
									
									
									, DriveOrigin.RobotCentric),   //Drive robot  
									new HolonomicPathFollowerConfig(
                    				new PIDConstants(.011, 0.005, 0	 ), // Translation PID constants
                    				new PIDConstants(.0009, 4.3, 0.7	), // Rotation PID constants
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
		//LOCK D KELLOG
		// FREE J HAUS
		swerveSubsystem.setDefaultCommand(fieldCentricCommand);
		//Set your auto
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("AutoChooser", autoChooser);
	}

	private void configureBindings() {
		driverBinds.zeroGyroTrigger().whileTrue(new InstantCommand(() -> swerveSubsystem.resetPoseToVision()));

		driverBinds.autoAimAndShootTrigger().whileTrue(autoAimAndShoot);


		manipulatorBinds.camTrigger().whileTrue(camCommand);

		Constants.rotationJoystick.button(1).whileTrue(autoAim);

		manipulatorBinds.climbRotateTenTimes().whileTrue(climbBalance);

		manipulatorBinds.climbWithJoySticks().whileTrue(climbAtSpeed);

		Constants.rotationJoystick.button(2).whileTrue(driveToRedSource);


		manipulatorBinds.shootTrigger().whileTrue(shootCommand);
		Constants.xbox.leftTrigger().whileTrue(new InstantCommand(() -> {shooterSubsystem.setFlywheelVoltage(5,5);
        
			if(Constants.xbox.getLeftY() < -0.5) {
				shooterSubsystem.setIntakeVoltage(9);
			} else {
				shooterSubsystem.setIntakeVoltage(0);
			}})).whileFalse(new InstantCommand(() -> shooterSubsystem.setFlywheelVoltage(0, 0)));

		manipulatorBinds.intakeTrigger().onTrue(new InstantCommand(() -> {
			shooterSubsystem.setIntakeVoltage(10);
		})).onFalse(new InstantCommand(() -> {
			shooterSubsystem.setIntakeVoltage(0);
		}));

		manipulatorBinds.exhaustTrigger().whileTrue(new InstantCommand(() -> {
			shooterSubsystem.setIntakeVoltage(-2);
		})).onFalse(new InstantCommand(() -> {
			shooterSubsystem.setIntakeVoltage(0);
		}));
		
		Constants.xbox.b().whileTrue(new InstantCommand(() -> shooterSubsystem.setCamPos(SmartDashboard.getNumber("SetCamPos", .5))));
		Constants.xbox.pov(0).onTrue(new InstantCommand(() -> shooterSubsystem.setCamPos(ShooterSubsystem.camLowerLimit)));
		Constants.xbox.pov(180).onTrue(new InstantCommand(() -> shooterSubsystem.setCamPos(ShooterSubsystem.camUpperLimit)));
		Constants.rotationJoystick.button(10).whileTrue(zeroYaw);

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
		onEnableCallback.onTrue(new InstantCommand(() -> {
			manipulatorBinds = manipulatorChooser.getSelected();
			driverBinds = driverChooser.getSelected();
		}));
		//onEnableCallback.onTrue(new InstantCommand(() -> swerveSubsystem.resetPoseToVision()));
		
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
