package frc.robot.subsystems;

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

	SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Constants.gyro.getRotation2d(),
	new SwerveModulePosition[] {
		frontLeftModule.getPosition(), frontRightModule.getPosition(),
		rearLeftModule.getPosition(), rearRightModule.getPosition()
	});

	public void drive(double xSpeed, double ySpeed, double zRotation, boolean fieldRelative) {

		System.out.println(xSpeed);
		ChassisSpeeds speeds = ChassisSpeeds.discretize(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zRotation, Constants.gyro.getRotation2d())
				: new ChassisSpeeds(xSpeed, ySpeed, zRotation),
			RobotContainer.getPeriod.getAsDouble());

		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);

		frontLeftModule.setDesiredState(states[0]);
		frontRightModule.setDesiredState(states[1]);
		rearLeftModule.setDesiredState(states[2]);
		rearRightModule.setDesiredState(states[3]);
	}

	// TODO: use update odometry
	public void updateOdometry() {
		odometry.update(Constants.gyro.getRotation2d(),
		new SwerveModulePosition[] {
			frontLeftModule.getPosition(), frontRightModule.getPosition(),
			rearLeftModule.getPosition(), rearRightModule.getPosition()
		});
	}
}