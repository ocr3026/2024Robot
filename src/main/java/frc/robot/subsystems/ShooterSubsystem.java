package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    
    CANcoder camEncoder = new CANcoder(44);
    CANSparkMax camMotor = new CANSparkMax(41, MotorType.kBrushless);
    PIDController camPID = new PIDController(7.5, .9, 0);

    public static final double camUpperLimit = .83264;
    public static final double camLowerLimit = .659404;

    SlewRateLimiter leftLimiter = new SlewRateLimiter(12 * 3);
    SlewRateLimiter rightLimiter = new SlewRateLimiter(12 * 3);

    double camTarget = camUpperLimit -.1;

    CANSparkMax intakeMotor = new CANSparkMax(21, MotorType.kBrushless);

    CANSparkMax leftFlywheel = new CANSparkMax(22, MotorType.kBrushless);
    RelativeEncoder leftEncoder = leftFlywheel.getEncoder(Type.kHallSensor, 42);
    CANSparkMax rightFlywheel = new CANSparkMax(23, MotorType.kBrushless);
    RelativeEncoder rightEncoder = rightFlywheel.getEncoder(Type.kHallSensor, 42);

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
        rightFlywheel.setVoltage(rightLimiter.calculate(rightVoltage));
        leftFlywheel.setVoltage(leftLimiter.calculate(leftVoltage));
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
        if(DriverStation.isEnabled());
            speed = camPID.calculate(camEncoder.getAbsolutePosition().getValueAsDouble(), MathUtil.clamp(camTarget, camLowerLimit, camUpperLimit));
        
            camMotor.set(MathUtil.clamp(speed, -.6, .6));
        

    
        SmartDashboard.putNumber("Cam Position", camEncoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Cam Target", camTarget);
        SmartDashboard.putNumber("Motor speed", speed);

        SmartDashboard.putNumber("leftSpeed", leftEncoder.getVelocity());
        SmartDashboard.putNumber("rightSpeed", rightEncoder.getVelocity());
        SmartDashboard.putBoolean("revvedUP", rightEncoder.getVelocity() >= 5300);
        
    }
} 

