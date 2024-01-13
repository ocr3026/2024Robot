package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
	SwerveDriveKinematics kinematics =
		new SwerveDriveKinematics(Constants.frontLeftModulePos, Constants.rearLeftModulePos,
	                              Constants.frontRightModulePos, Constants.rearRightModulePos);

	SwerveModule rearRightModule = new SwerveModule(7, 8, 10, true);
	SwerveModule frontRightModule = new SwerveModule(5, 6, 12, false);
	SwerveModule rearLeftModule = new SwerveModule(3, 4, 11, false);
	public SwerveModule frontLeftModule = new SwerveModule(1, 2, 9, true);

	public void driveRobotCentric(double xSpeed, double ySpeed, double zRotation, boolean fieldRelative) {
		ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, -ySpeed, -zRotation / 48);

		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

		double max = 1;
		for (SwerveModuleState i : states) {
			if (Math.abs(i.speedMetersPerSecond) > max) {
				max = Math.abs(i.speedMetersPerSecond);
			}
		}

		for (SwerveModuleState i : states) {
			i.speedMetersPerSecond /= max;
		}

		frontLeftModule.setDesiredState(states[0]);
		rearLeftModule.setDesiredState(states[1]);
		frontRightModule.setDesiredState(states[2]);
		rearRightModule.setDesiredState(states[3]);
	}
}