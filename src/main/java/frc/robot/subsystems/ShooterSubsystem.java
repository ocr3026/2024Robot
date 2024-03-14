package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    CANcoder camEncoder = new CANcoder(44);
    CANSparkMax camMotor = new CANSparkMax(45, MotorType.kBrushless);
    PIDController camPID = new PIDController(0, 0, 0);

    SlewRateLimiter smoothCurrent = new SlewRateLimiter(3);

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

    public void setCamDegrees(double position) {
        position = MathUtil.clamp(position, 0, 360);
        camMotor.setVoltage(camPID.calculate(camEncoder.getAbsolutePosition().getValueAsDouble(), position / 360));
    }

    public double getCamDegrees() {
        return camEncoder.getAbsolutePosition().getValueAsDouble() * 360;
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
        SmartDashboard.putNumber("Cam Position", getCamDegrees());

        if(smoothCurrent.calculate(camMotor.getOutputCurrent()) > 40) {
            camMotor.disable();
            System.out.println("Disabled Cam Due to overcurrent");
        }
    }
}
