package frc.robot.util;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class SwerveModule {
	CANSparkMax driveMotor;
	public RelativeEncoder driveEncoder;
	CANSparkMax steerMotor;
	CANcoder steerEncoder;

	public PIDController drivePID = new PIDController(0, 0, 0);

	public PIDController steerPID = new PIDController(0.166, 0, 0.000012);

	public SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(.14, 2.59, 0.48);

	public SwerveModule(int driveMotorID, int steerMotorID, int encoderID) {
		driveMotor = new CANSparkMax(driveMotorID, CANSparkMax.MotorType.kBrushless);
		steerMotor = new CANSparkMax(steerMotorID, CANSparkMax.MotorType.kBrushless);

		driveMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.setInverted(false);

		driveEncoder = driveMotor.getEncoder(Type.kHallSensor, 42);
		driveEncoder.setVelocityConversionFactor(2 * Math.PI * Constants.wheelRadius / (Constants.gearRatio * 60));
		driveEncoder.setPositionConversionFactor(2 * Math.PI * Constants.wheelRadius / Constants.gearRatio);

		steerMotor.setIdleMode(IdleMode.kBrake);

		steerEncoder = new CANcoder(encoderID);

		steerPID.enableContinuousInput(-180, 180);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValueAsDouble()));
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValueAsDouble()));
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		Rotation2d curSteerAngle = Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValueAsDouble());

		SwerveModuleState state = SwerveModuleState.optimize(desiredState, curSteerAngle);

		state.speedMetersPerSecond *= Math.pow(state.angle.minus(curSteerAngle).getCos(), 3);

		double driveFB = drivePID.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
		double driveFF = driveFeedForward.calculate(state.speedMetersPerSecond);

		double steerFB = steerPID.calculate(curSteerAngle.getDegrees(), state.angle.getDegrees());

		if(steerMotor.getDeviceId() == 4) {
			SmartDashboard.putNumber("Target drive velocity", state.speedMetersPerSecond);
			SmartDashboard.putNumber("Current drive velocity", driveEncoder.getVelocity());
			SmartDashboard.putNumber("Drive position", driveEncoder.getPosition());
		}

		driveMotor.setVoltage(driveFB + driveFF);
		steerMotor.setVoltage(steerFB);
	}

	public void setTuna() {
		driveFeedForward = new SimpleMotorFeedforward(
			SmartDashboard.getNumber("driveKs", 0), 
			SmartDashboard.getNumber("driveKv", 0), 
			SmartDashboard.getNumber("driveKa", 0));

		drivePID = new PIDController(
			SmartDashboard.getNumber("driveP", 0), 
			SmartDashboard.getNumber("driveI", 0), 
			SmartDashboard.getNumber("driveD", 0));

		steerPID = new PIDController(
			SmartDashboard.getNumber("steerP", 0), 
			SmartDashboard.getNumber("steerI", 0), 
			SmartDashboard.getNumber("steerD", 0));

		steerPID.enableContinuousInput(-180, 180);
	}
}