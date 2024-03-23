package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    CANcoder camEncoder = new CANcoder(44);
    CANSparkMax camMotor = new CANSparkMax(41, MotorType.kBrushless);
    PIDController camPID = new PIDController(8, 0.5, 0);

    public static final double camUpperLimit = 0.671;
    public static final double camLowerLimit = 0.471;

    double camTarget = 0.671;

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

        camPID.setTolerance(0.01);
        //SmartDashboard.putNumber("set cam pos", 0);
    }

    public void setCamPos(double position) {
        camTarget = position;
    }

    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public void setFlywheelVoltage(double leftVoltage, double rightVoltage) {
        rightFlywheel.setVoltage(rightVoltage);
        leftFlywheel.setVoltage(leftVoltage);
    }
        public void setFlywheelSpeed(double leftVoltage, double rightVoltage) {
        rightFlywheel.set(MathUtil.clamp(rightVoltage,0,1));
        leftFlywheel.set(MathUtil.clamp(leftVoltage,0,1));
    }
    public double getFlywheelSpeed () {
       return  rightFlywheel.get();
    }

    @Override
    public void periodic() {
        double speed = 0;
        //camTarget = SmartDashboard.getNumber("set cam pos", 0);
        if(DriverStation.isEnabled())
            speed = camPID.calculate(camEncoder.getAbsolutePosition().getValueAsDouble(), MathUtil.clamp(camTarget, camLowerLimit, camUpperLimit));
        
        camMotor.set(speed);

        SmartDashboard.putNumber("Cam Position", camEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Cam Target", camTarget);
        SmartDashboard.putNumber("Motor speed", speed);
    }
}

