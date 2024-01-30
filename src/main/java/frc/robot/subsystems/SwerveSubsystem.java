package frc.robot.subsystems;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.SwerveModule;


public class SwerveSubsystem extends SubsystemBase {
	SwerveDriveKinematics kinematics =
		new SwerveDriveKinematics(Constants.frontLeftModulePos, Constants.frontRightModulePos,
	                              Constants.rearLeftModulePos, Constants.rearRightModulePos);

	public SwerveModule frontLeftModule = new SwerveModule(5, 6, 12);
	public SwerveModule rearRightModule = new SwerveModule(3, 4, 11);
	public SwerveModule rearLeftModule = new SwerveModule(1, 2, 9);
	public SwerveModule frontRightModule = new SwerveModule(7, 8, 10);

	SwerveDriveOdometry odometry;

	public SwerveSubsystem() {
		Constants.gyro.reset();

		Constants.gyro.setGyroAngle(Constants.gyro.getYawAxis(), Constants.initialYaw);

		odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(Constants.gyro.getAngle(Constants.gyro.getYawAxis())),
			new SwerveModulePosition[] {
				frontLeftModule.getPosition(), frontRightModule.getPosition(),
				rearLeftModule.getPosition(), rearRightModule.getPosition()
			});
	}

	public void drive(double xSpeed, double ySpeed, double zRotation, boolean fieldRelative) {
		ChassisSpeeds speeds = ChassisSpeeds.discretize(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zRotation, Rotation2d.fromDegrees(Constants.gyro.getAngle(Constants.gyro.getYawAxis())))
				: new ChassisSpeeds(xSpeed, ySpeed, zRotation),
			RobotContainer.getPeriod.getAsDouble());

		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);

		frontLeftModule.setDesiredState(states[0]);
		frontRightModule.setDesiredState(states[1]);
		rearLeftModule.setDesiredState(states[2]);
		rearRightModule.setDesiredState(states[3]);
	}

	public void updateOdometry() {
		odometry.update(Rotation2d.fromDegrees(Constants.gyro.getAngle(Constants.gyro.getYawAxis())),
		new SwerveModulePosition[] {
			frontLeftModule.getPosition(), frontRightModule.getPosition(),
			rearLeftModule.getPosition(), rearRightModule.getPosition()
		});
	}
	public Pose2d getPose () {
		return odometry.getPoseMeters();
	}
	public void resetPose (Pose2d poser) {
		odometry.resetPosition(Rotation2d.fromDegrees(Constants.gyro.getAngle(Constants.gyro.getYawAxis())), new SwerveModulePosition[] {
			frontLeftModule.getPosition(), frontRightModule.getPosition(),
			rearLeftModule.getPosition(), rearRightModule.getPosition()
		}, poser);
	}
	public ChassisSpeeds speedGetter (double xSpeed, double ySpeed, double zRotation) {
		return new ChassisSpeeds(xSpeed, ySpeed, zRotation);
	}
	

	@Override
	public void periodic() {
		updateOdometry();
	}
}