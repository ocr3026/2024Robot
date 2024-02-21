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
    RelativeEncoder leftEncoder = leftFlywheel.getEncoder(Type.kHallSensor, 42);
    SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(0.09, 0.0021, 0);
    PIDController leftFB = new PIDController(0, 0, 0);

    CANSparkMax rightFlywheel = new CANSparkMax(23, MotorType.kBrushless);
    RelativeEncoder rightEncoder = rightFlywheel.getEncoder(Type.kHallSensor, 42);
    SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(0.09, 0.0021, 0);
    PIDController rightFB = new PIDController(0, 0, 0);

    public ShooterSubsystem() {
        leftActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        rightActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

        leftFB.setTolerance(200);
        rightFB.setTolerance(200);

        rightFlywheel.setInverted(false);
        leftFlywheel.setInverted(true);
        intakeMotor.setInverted(false);

        rightFlywheel.setIdleMode(IdleMode.kCoast);
        leftFlywheel.setIdleMode(IdleMode.kCoast);
        intakeMotor.setIdleMode(IdleMode.kCoast);
    }
    public void setSpeed(double speed) {
        rightActuator.setSpeed(speed);
        leftActuator.setSpeed(speed);
    }

    public void setAngle(Rotation2d angle) {
        targetAngle = angle;
    }
    public void toAngle () {
        targetServoLength = ((targetAngle.getDegrees() -25) * (maxServoLength/20));
        leftActuator.set(targetServoLength);
        rightActuator.set(targetServoLength);

        
    }

    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }
    

    /**
     * Sets the speed of the dual flywheel shooter in rotations per second (rpm)
     * @param leftAngularVelocity angular velocity of the left flywheel in rpm
     * @param rightAngularVelocity angular velocity of the right flywheel in rpm
     */
    public void setFlywheelSpeeds(double leftAngularVelocity, double rightAngularVelocity) {
        SmartDashboard.putNumber("currentSpeed", leftEncoder.getVelocity());

        double leftCalculatedFF = leftFF.calculate(leftAngularVelocity);
        double leftCalculatedFB = leftFB.calculate(leftEncoder.getVelocity(), leftAngularVelocity);
        leftFlywheel.setVoltage(leftCalculatedFF + leftCalculatedFB);

        double rightCalculatedFF = rightFF.calculate(rightAngularVelocity);
        double rightCalculatedFB = rightFB.calculate(rightEncoder.getVelocity(), rightAngularVelocity);
        rightFlywheel.setVoltage(rightCalculatedFF + rightCalculatedFB);
    }

    public void setFlywheelVoltage(double leftVoltage, double rightVoltage) {
        rightFlywheel.setVoltage(rightVoltage);
        leftFlywheel.setVoltage(leftVoltage);
    }

    public boolean areFlywheelsSpunUp() {
        return leftFB.atSetpoint() && rightFB.atSetpoint();
    }

    @Override
    public void periodic() {

    }
}
