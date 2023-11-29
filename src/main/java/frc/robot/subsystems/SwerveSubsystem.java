package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
	SwerveDriveKinematics kinematics =
		new SwerveDriveKinematics(Constants.frontLeftModulePos, Constants.rearLeftModulePos,
	                              Constants.frontRightModulePos, Constants.rearRightModulePos);

	PIDController PID = new PIDController(0, 0, 0);
	PIDController balancePID = new PIDController(0, 0, 0);

	SwerveModule rearRightModule = new SwerveModule(7, 8, 10, 0.018, 0, 0.000001, true);
	SwerveModule frontRightModule = new SwerveModule(5, 6, 12, 0.018, 0, 0.000001, false);
	SwerveModule rearLeftModule = new SwerveModule(3, 4, 11, 0.018, 0, 0.000001, false);
	public SwerveModule frontLeftModule = new SwerveModule(1, 2, 9, 0.018, 0, 0.000001, true);

	public SwerveSubsystem () {
		SmartDashboard.putNumber("wheel", frontLeftModule.driveEncoder.getPosition());
	}

	

	public void driveRobotCentric(double xSpeed, double ySpeed, double zRotation) {
		SmartDashboard.putNumber("wheel", frontLeftModule.driveEncoder.getPosition());
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

		frontLeftModule.setState(states[0]);
		rearLeftModule.setState(states[1]);
		frontRightModule.setState(states[2]);
		rearRightModule.setState(states[3]);
	}

	public void driveFieldCentric(double xSpeed, double ySpeed, double zRotation) {
		SmartDashboard.putNumber("wheel", frontLeftModule.driveEncoder.getPosition());
		ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			xSpeed, -ySpeed, -zRotation / 48, Rotation2d.fromDegrees(-Constants.gyro.getYaw()));

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

		frontLeftModule.setState(states[0]);
		rearLeftModule.setState(states[1]);
		frontRightModule.setState(states[2]);
		rearRightModule.setState(states[3]);
	}

	public Command OnDisableCommand() {
		return new InstantCommand(() -> {
			frontLeftModule.resetEncoder();
			rearLeftModule.resetEncoder();
			frontRightModule.resetEncoder();
			rearRightModule.resetEncoder();
		});
	}

	public void setBrake() {
		frontLeftModule.setBrake();
		rearLeftModule.setBrake();
		frontRightModule.setBrake();
		rearRightModule.setBrake();
	}

	public void setCoast() {
		frontLeftModule.setCoast();
		rearLeftModule.setCoast();
		frontRightModule.setCoast();
		rearRightModule.setCoast();
	}
	public  Command zeroWheel () {
		return new InstantCommand( () -> frontLeftModule.driveEncoder.setPosition(0));
	
	}
	public void zeroWheelFunc () {
		frontLeftModule.driveEncoder.setPosition(0);
	}
}