package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.util.AHRSMixin;

public final class Constants {
	// Gyro
	public static final AHRSMixin gyro = new AHRSMixin(SerialPort.Port.kMXP);
	
	// Swerve Drive Constants
	public static final double maxSpeed = 3; // m/s
	public static final double maxAngularSpeed = Math.PI; // rad/s
	public static final double wheelRadius = 0.0508; // m
	// public static final double gearRatio = 6.12; // L3
	public static final double gearRatio = 6.75; // L2
	public static final int neoCountsPerRevolution = 42;

	// Swerve Module Positions
	public static final Translation2d frontLeftModulePos = new Translation2d(0.36195, -0.36195);
	public static final Translation2d rearLeftModulePos = new Translation2d(-0.36195, -0.36195);
	public static final Translation2d frontRightModulePos = new Translation2d(0.36195, 0.36195);
	public static final Translation2d rearRightModulePos = new Translation2d(-0.36195, 0.36195);

	// Deadband
	public static final double deadband = 0.1;

	// Controls
	public static final CommandJoystick translationJoystick = new CommandJoystick(0);
	public static final CommandJoystick rotationJoystick = new CommandJoystick(1);

	public static final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
	public static final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
	public static final SlewRateLimiter zRotSpeedLimiter = new SlewRateLimiter(3);
}
