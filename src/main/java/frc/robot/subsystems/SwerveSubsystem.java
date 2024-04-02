package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.SwerveModule;


public class SwerveSubsystem extends SubsystemBase {
	
	SwerveDriveKinematics kinematics =
		new SwerveDriveKinematics(Constants.frontLeftModulePos, Constants.frontRightModulePos,
	                              Constants.rearLeftModulePos, Constants.rearRightModulePos);

	public SwerveModule frontLeftModule = new SwerveModule(5, 6, 10);
	public SwerveModule rearRightModule = new SwerveModule(4, 3, 12);
	public SwerveModule rearLeftModule = new SwerveModule(1, 2, 9);
	public SwerveModule frontRightModule = new SwerveModule(7, 8, 11);

	SwerveDrivePoseEstimator odometry;
	Pose2d robotPose = new Pose2d();
	Timer timer = new Timer();
	public ADIS16470_IMU gyro = new ADIS16470_IMU();

	SwerveModuleState[] states = null;

	public SwerveSubsystem() {
		 SmartDashboard.putBoolean("inRange", false);
		gyro.reset();

		odometry = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis())),
			new SwerveModulePosition[] {
				frontLeftModule.getPosition(), frontRightModule.getPosition(),
				rearLeftModule.getPosition(), rearRightModule.getPosition()
			}, robotPose);

		timer.restart();
	}

	public void drive(double xSpeed, double ySpeed, double zRotation, DriveOrigin driveOrigin) {
		SmartDashboard.putNumber("xSpeed", xSpeed);
		SmartDashboard.putNumber("ySpeed", ySpeed);
		SmartDashboard.putNumber("zRotation", zRotation);
		ChassisSpeeds speeds;
		switch(driveOrigin) {
			case AllianceCentric:
				if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
					speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zRotation, robotPose.getRotation().plus(Rotation2d.fromDegrees(180)).unaryMinus());
					break;
				}
				speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zRotation, robotPose.getRotation().unaryMinus());
				break;
			case FieldCentric:
				speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zRotation, robotPose.getRotation().unaryMinus());
				break;
			case RobotCentric:
				speeds = new ChassisSpeeds(xSpeed, ySpeed, zRotation);
				break;
			default:
				speeds = new ChassisSpeeds(xSpeed, ySpeed, zRotation);
				break;
		}

		states = kinematics.toSwerveModuleStates(speeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);
	}

	public void updateOdometry() {
		/*
		if(Constants.visionPoseEstimator.isPresent()) {
			Constants.visionPoseEstimator.get().setReferencePose(robotPose);
			var visionPose = Constants.visionPoseEstimator.get().update();
			if(visionPose.isPresent()) {
				odometry.addVisionMeasurement(visionPose.get().estimatedPose.toPose2d(), timer.get());
			}
		}
		*/
		
		robotPose = odometry.updateWithTime(timer.get(), Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis())),
		new SwerveModulePosition[] {
			frontLeftModule.getPosition(), frontRightModule.getPosition(),
			rearLeftModule.getPosition(), rearRightModule.getPosition()
		});
	}
	
	public void resetPose (Pose2d poser) {
		gyro.setGyroAngle(gyro.getYawAxis(), poser.getRotation().getDegrees());

		odometry.resetPosition(Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis())), new SwerveModulePosition[] {
			frontLeftModule.getPosition(), frontRightModule.getPosition(),
			rearLeftModule.getPosition(), rearRightModule.getPosition()
		}, poser);
	}

	public void autoResetPose (Pose2d poser) {
		gyro.setGyroAngle(gyro.getYawAxis(), poser.getRotation().getDegrees());

		odometry.resetPosition(Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis())), new SwerveModulePosition[] {
			frontLeftModule.getPosition(), frontRightModule.getPosition(),
			rearLeftModule.getPosition(), rearRightModule.getPosition()
		}, poser);
	}

	public void resetPoseToVision() {
		/*if(Constants.visionPoseEstimator.isPresent()) {
			var visionPose = Constants.visionPoseEstimator.get().update();
			if(visionPose.isPresent()) {
				gyro.setGyroAngle(gyro.getYawAxis(), visionPose.get().estimatedPose.getRotation().toRotation2d().getDegrees());

				odometry.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle(gyro.getYawAxis())), new SwerveModulePosition[] {
				frontLeftModule.getPosition(), frontRightModule.getPosition(),
				rearLeftModule.getPosition(), rearRightModule.getPosition()
				}, visionPose.get().estimatedPose.toPose2d());
			} else {
				resetPose(new Pose2d());
			}
		} else {
			*/resetPose(new Pose2d());/*
		}*/
	}

	public Pose2d getPose() {


		return robotPose;
	}

	public Pose2d autoGetPose() {
		return robotPose;
	}
	
	public ChassisSpeeds speedGetter () {
		return kinematics.toChassisSpeeds(frontLeftModule.getState(), frontRightModule.getState(),
										   rearLeftModule.getState(), rearRightModule.getState());
	}

	public void drive(ChassisSpeeds speeds) {
		SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);
	}
	
	public Rotation2d getGyroRoll() {
		return Rotation2d.fromDegrees(gyro.getAngle(gyro.getRollAxis()));
	}

	@Override
	public void periodic() {
		PhotonTrackedTarget target = null;
		if(Constants.camera.isPresent()) {
			PhotonCamera camera = Constants.camera.get();

            target = camera.getLatestResult().getBestTarget();
            if(target != null) {
                SmartDashboard.putBoolean("inRange", target.getBestCameraToTarget().getX() <= 3.7);
        }
		else {
			 SmartDashboard.putBoolean("inRange", false);
		}
        }
		if(states != null) {
			frontLeftModule.setDesiredState(states[0]);
			frontRightModule.setDesiredState(states[1]);
			rearLeftModule.setDesiredState(states[2]);
			rearRightModule.setDesiredState(states[3]);
		}
		updateOdometry();
		SmartDashboard.putNumber("robotX", getPose().getX());
		SmartDashboard.putNumber("robotY", getPose().getY());
		SmartDashboard.putNumber("robotYaw", getPose().getRotation().getDegrees());
		SmartDashboard.putNumber("gyro roll", gyro.getAngle(gyro.getRollAxis()));
		
	}

	public enum DriveOrigin {
		RobotCentric,	 // Origin facing the robot's forward
		AllianceCentric, // Origin facing away from the alliance wall you are on
		FieldCentric	 // Origin facing towards red alliance
	}
}