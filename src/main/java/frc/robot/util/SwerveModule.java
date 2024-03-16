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

	// TODO: Add PID to account for voltage drop on assualt and battery
	public PIDController drivePID = new PIDController(0, 0, 0);

	public PIDController steerPID = new PIDController(0.166, 0, 0.000012);

	// TODO: Find kS
	// Calculated with ReCalc
	// https://www.reca.lc/drive?appliedVoltageRamp=%7B%22s%22%3A1200%2C%22u%22%3A%22V%2Fs%22%7D&batteryAmpHours=%7B%22s%22%3A18%2C%22u%22%3A%22A%2Ah%22%7D&batteryResistance=%7B%22s%22%3A0.018%2C%22u%22%3A%22Ohm%22%7D&batteryVoltageAtRest=%7B%22s%22%3A12.5%2C%22u%22%3A%22V%22%7D&efficiency=97&filtering=1&gearRatioMax=%7B%22magnitude%22%3A15%2C%22ratioType%22%3A%22Reduction%22%7D&gearRatioMin=%7B%22magnitude%22%3A3%2C%22ratioType%22%3A%22Reduction%22%7D&maxSimulationTime=%7B%22s%22%3A4%2C%22u%22%3A%22s%22%7D&maxSpeedAccelerationThreshold=%7B%22s%22%3A0.15%2C%22u%22%3A%22ft%2Fs2%22%7D&motor=%7B%22quantity%22%3A4%2C%22name%22%3A%22NEO%22%7D&motorCurrentLimit=%7B%22s%22%3A60%2C%22u%22%3A%22A%22%7D&numCyclesPerMatch=24&peakBatteryDischarge=20&ratio=%7B%22magnitude%22%3A6.12%2C%22ratioType%22%3A%22Reduction%22%7D&sprintDistance=%7B%22s%22%3A25%2C%22u%22%3A%22ft%22%7D&swerve=1&targetTimeToGoal=%7B%22s%22%3A2%2C%22u%22%3A%22s%22%7D&throttleResponseMax=0.99&throttleResponseMin=0.5&weightAuxilliary=%7B%22s%22%3A28%2C%22u%22%3A%22lbs%22%7D&weightDistributionFrontBack=0.5&weightDistributionLeftRight=0.5&weightInspected=%7B%22s%22%3A72%2C%22u%22%3A%22lbs%22%7D&wheelBaseLength=%7B%22s%22%3A27%2C%22u%22%3A%22in%22%7D&wheelBaseWidth=%7B%22s%22%3A20%2C%22u%22%3A%22in%22%7D&wheelCOFDynamic=0.9&wheelCOFLateral=1.1&wheelCOFStatic=1.1&wheelDiameter=%7B%22s%22%3A4%2C%22u%22%3A%22in%22%7D
	public SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(.14, 2.59, 0.4);

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