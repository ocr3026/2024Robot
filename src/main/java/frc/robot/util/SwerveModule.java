package frc.robot.util;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;


public class SwerveModule {
	CANSparkMax driveMotor;
	RelativeEncoder driveEncoder;
	CANSparkMax steerMotor;
	CANcoder steerEncoder;

	public static final double moduleMaxAngularVelocity = Constants.maxAngularSpeed; // rad/s
	public static final double moduleMaxAngularAcceleration = 2 * Math.PI; // rad/s^2

	PIDController drivePID = new PIDController(0, 0, 0);

	ProfiledPIDController steerPID = new ProfiledPIDController(0, 0, 0, 
		new TrapezoidProfile.Constraints(moduleMaxAngularVelocity, moduleMaxAngularAcceleration));

	SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(1, 3, 0);
	SimpleMotorFeedforward steerFeedForward = new SimpleMotorFeedforward(1, 0.5, 0);

	public SwerveModule(int driveMotorID, int steerMotorID, int encoderID, boolean inverted) {
		driveMotor = new CANSparkMax(driveMotorID, CANSparkMax.MotorType.kBrushless);
		steerMotor = new CANSparkMax(steerMotorID, CANSparkMax.MotorType.kBrushless);

		driveMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.setInverted(inverted);

		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(2 * Math.PI * Constants.wheelRadius / Constants.gearRatio);
		driveEncoder.setVelocityConversionFactor(2 * Math.PI * Constants.wheelRadius / Constants.gearRatio);

		steerMotor.setIdleMode(IdleMode.kBrake);

		steerEncoder = new CANcoder(encoderID);

		steerPID.enableContinuousInput(-Math.PI, Math.PI);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition().getValueAsDouble()));
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition().getValueAsDouble()));
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		Rotation2d curSteerAngle = Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition().getValueAsDouble());

		SwerveModuleState state = SwerveModuleState.optimize(desiredState, curSteerAngle);

		state.speedMetersPerSecond *= state.angle.minus(curSteerAngle).getCos();

		double driveFB = drivePID.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
		double driveFF = driveFeedForward.calculate(state.speedMetersPerSecond);

		double steerFB = steerPID.calculate(steerEncoder.getAbsolutePosition().getValueAsDouble(), state.angle.getDegrees());
		double steerFF = steerFeedForward.calculate(steerPID.getSetpoint().velocity);

		driveMotor.setVoltage(driveFB + driveFF);
		steerMotor.setVoltage(steerFB + steerFF);
	}
}