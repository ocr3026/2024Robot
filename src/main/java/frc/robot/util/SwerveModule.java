package frc.robot.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
	CANSparkMax driveMotor;
	public RelativeEncoder driveEncoder;
	CANSparkMax steerMotor;
	CANCoder steerEncoder;
	PIDController steerPID;

	public SwerveModule(int driveMotorID, int steerMotorID, int encoderID, double p, double i,
	                    double d, boolean inverted) {
		driveMotor = new CANSparkMax(driveMotorID, CANSparkMax.MotorType.kBrushless);
		steerMotor = new CANSparkMax(steerMotorID, CANSparkMax.MotorType.kBrushless);

		driveMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.setInverted(inverted);

		driveEncoder = driveMotor.getEncoder();

		steerMotor.setIdleMode(IdleMode.kBrake);

		steerEncoder = new CANCoder(encoderID);

		steerPID = new PIDController(p, i, d);
		steerPID.enableContinuousInput(-180, 180);
		
	}

	public void setState(SwerveModuleState state) {
		Rotation2d curSteerAngle = Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition());

		SwerveModuleState newState = SwerveModuleState.optimize(state, curSteerAngle);

		driveMotor.set(
			newState.speedMetersPerSecond *
			Math.pow(Math.cos(newState.angle.getRadians() - curSteerAngle.getRadians()), 2));

		steerMotor.set(steerPID.calculate(curSteerAngle.getDegrees(), newState.angle.getDegrees()));
		
	}

	public void resetEncoder() { System.out.println(steerEncoder.setPositionToAbsolute()); }

	public void setCoast() { driveMotor.setIdleMode(IdleMode.kCoast); }

	public void setBrake() { driveMotor.setIdleMode(IdleMode.kBrake); }
}