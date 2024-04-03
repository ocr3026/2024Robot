package frc.robot;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Constants {

	//Points
	// Swerve Drive Constants
	public static final double maxSpeed = 5; // m/s
	public static final double maxAngularSpeed = Math.PI /1; // rad/s
	public static final double wheelRadius = 0.0508; // m
	
	public static final int neoCountsPerRevolution = 42;

	// Swerve Drive Config
	
	// Tuna
	public static final double gearRatio = 6.12; // L3
	public static final Translation2d frontLeftModulePos = new Translation2d(0.23495, -0.23495);
	public static final Translation2d rearLeftModulePos = new Translation2d(-0.23495, -0.23495);
	public static final Translation2d frontRightModulePos = new Translation2d(0.23495, 0.23495);
	public static final Translation2d rearRightModulePos = new Translation2d(-0.23495, 0.23495);

	// Rico
	/*
	public static final double gearRatio = 6.75; // L2
	public static final Translation2d frontLeftModulePos = new Translation2d(0.3, -0.3);
	public static final Translation2d rearLeftModulePos = new Translation2d(-0.3, -0.3);
	public static final Translation2d frontRightModulePos = new Translation2d(0.3, 0.3);
	public static final Translation2d rearRightModulePos = new Translation2d(-0.3, 0.3);
	*/
	
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
	public static Optional<PhotonCamera> camera = Optional.empty();

	// Shooting curve calculated from Russell's sexy desmos graph. 🥵🥵🥵🥵🥵

	public static final double a = 0.00245173, b = -.0560599, c = .384591, d = 0	;  // ax^3 + bx^2 + cx + d
	//	public static final double a = -0.0346106, b = 0.348084, c = -1.08101, d = 1.63729; // ax^3 + bx^2 + cx + d
//"(K)eep (Y)ourself (S)afe"
}
