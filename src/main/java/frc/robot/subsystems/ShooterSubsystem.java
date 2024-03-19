package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    CANcoder camEncoder = new CANcoder(44);
    CANSparkMax camMotor = new CANSparkMax(41, MotorType.kBrushless);
    PIDController camPID = new PIDController(3, .02, 0);

    double camTarget = 0.671631;

    final double camUpperLimit = 0.671631;
    final double camLowerLimit = 0.513867;

    CANSparkMax intakeMotor = new CANSparkMax(21, MotorType.kBrushless);
    CANSparkMax leftFlywheel = new CANSparkMax(22, MotorType.kBrushless);
    CANSparkMax rightFlywheel = new CANSparkMax(23, MotorType.kBrushless);

    public ShooterSubsystem() {
        rightFlywheel.setInverted(false);
        leftFlywheel.setInverted(true);
        intakeMotor.setInverted(true);

        rightFlywheel.setIdleMode(IdleMode.kBrake);
        leftFlywheel.setIdleMode(IdleMode.kBrake);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setCamPos(double position) {
        camTarget = MathUtil.interpolate(camLowerLimit, camUpperLimit, 1 - position);
    }
    
    public void stopMotor () {
        camMotor.set(0);
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

        camMotor.set(camPID.calculate(camEncoder.getAbsolutePosition().getValueAsDouble(), MathUtil.clamp(camTarget, camLowerLimit, camUpperLimit)));

        SmartDashboard.putNumber("Cam Position", camEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Cam Target",  camTarget);
    }
}

