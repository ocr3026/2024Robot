package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    Rotation2d targetAngle = new Rotation2d();
    Servo leftActuator = new Servo(0);
    Servo rightActuator = new Servo(1);
    double targetServoLength = 0;
    double maxServoLength = 1;

    CANSparkMax intakeMotor = new CANSparkMax(21, MotorType.kBrushless);

    CANSparkMax leftFlywheel = new CANSparkMax(22, MotorType.kBrushless);

    CANSparkMax rightFlywheel = new CANSparkMax(23, MotorType.kBrushless);

    public ShooterSubsystem() {
        
        leftActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        rightFlywheel.setInverted(false);
        leftFlywheel.setInverted(true);
        intakeMotor.setInverted(true);

        rightFlywheel.setIdleMode(IdleMode.kBrake);
        leftFlywheel.setIdleMode(IdleMode.kBrake);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }
    public void setSpeed(double speed) {
        rightActuator.setSpeed(speed);
        leftActuator.setSpeed(speed);
        SmartDashboard.putNumber("Servo", rightActuator.getSpeed());
        SmartDashboard.putNumber("ServoPos", rightActuator.get());
    }

    public void setAngle(Rotation2d angle) {
        targetAngle = angle;
    }

    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public void setFlywheelVoltage(double leftVoltage, double rightVoltage) {
        rightFlywheel.setVoltage(rightVoltage);
        leftFlywheel.setVoltage(leftVoltage);
    }

    @Override
    public void periodic() {

    }
}
