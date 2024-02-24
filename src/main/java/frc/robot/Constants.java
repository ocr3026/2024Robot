package frc.robot;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Constants {
	// Swerve Drive Constants
	public static final double maxSpeed = 4; // m/s
	public static final double maxAngularSpeed = Math.PI; // rad/s
	public static final double wheelRadius = 0.0508; // m
	
	public static final int neoCountsPerRevolution = 42;

	public static boolean halfSpeed = false;

	// Swerve Drive Config
	
	// Tuna
	/*
	public static final double gearRatio = 6.12; // L3
	public static final Translation2d frontLeftModulePos = new Translation2d(0.36195, -0.36195);
	public static final Translation2d rearLeftModulePos = new Translation2d(-0.36195, -0.36195);
	public static final Translation2d frontRightModulePos = new Translation2d(0.36195, 0.36195);
	public static final Translation2d rearRightModulePos = new Translation2d(-0.36195, 0.36195);
	*/

	// Rico

	public static final double gearRatio = 6.75; // L2
	public static final Translation2d frontLeftModulePos = new Translation2d(0.3, -0.3);
	public static final Translation2d rearLeftModulePos = new Translation2d(-0.3, -0.3);
	public static final Translation2d frontRightModulePos = new Translation2d(0.3, 0.3);
	public static final Translation2d rearRightModulePos = new Translation2d(-0.3, 0.3);

	
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
	public static Optional<PhotonPoseEstimator> visionPoseEstimator = Optional.empty();
	public static final Transform3d robotToCamera = new Transform3d();
	public static final AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
}
