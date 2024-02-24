package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    PIDController leftPID = new PIDController(0, 0, 0);
    PIDController rightPID = new PIDController(0, 0, 0);
    CANSparkMax leftClimb = new  CANSparkMax(31, MotorType.kBrushless);
    CANSparkMax rightClimb = new CANSparkMax(32, MotorType.kBrushless);

    RelativeEncoder leftEncoder = leftClimb.getEncoder(Type.kHallSensor, 42);
    RelativeEncoder rightEncoder = rightClimb.getEncoder(Type.kHallSensor, 42);



    public ClimberSubsystem () {
        leftEncoder.setPositionConversionFactor(1);
        rightEncoder.setPositionConversionFactor(1);

        leftClimb.setInverted(true);

        leftClimb.setIdleMode(IdleMode.kBrake);
        rightClimb.setIdleMode(IdleMode.kBrake);
    }
     public void climbSpeed (double speed) {
        leftClimb.set(speed);
        rightClimb.set(speed);
    }

    public void climbSpeedRight(double speed) {
        rightClimb.set(speed);
    }
    public void climbSpeedLeft(double speed) {
        leftClimb.set(speed);
    }

    public void setLeftClimbPos(double setPos) {
        leftClimb.setVoltage(leftPID.calculate(leftEncoder.getPosition(), setPos));
    }

    public void setRightClimbPos(double setPos) {
        rightClimb.setVoltage(rightPID.calculate(rightEncoder.getPosition(), setPos));
    }

}
