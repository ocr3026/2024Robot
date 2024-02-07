package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Constants {
	// Gyro
	public static final ADIS16470_IMU gyro = new ADIS16470_IMU();
	public static final double initialYaw = 0;
	
	// TODO: Switch back to L3 for new robot
	// Swerve Drive Constants
	public static final double maxSpeed = 5; // m/s
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
	public static final CommandXboxController xbox = new CommandXboxController(2);

	public static final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
	public static final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
	public static final SlewRateLimiter zRotSpeedLimiter = new SlewRateLimiter(3);

	// Enable PID Tuning in the smartdashboard
	public static final boolean tunaFish = false;

	// Camera
	//public static final PhotonCamera camera = new PhotonCamera("photonvision");
	public static final Translation3d laserToCamera = new Translation3d(-0.03, -0.065, 0.225);
}
