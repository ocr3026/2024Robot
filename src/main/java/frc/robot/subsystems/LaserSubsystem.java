package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LaserSubsystem extends SubsystemBase {
    CANSparkMax aimMotor = new CANSparkMax(62, MotorType.kBrushless);
    RelativeEncoder aimEncoder = aimMotor.getEncoder(Type.kHallSensor, 42);

    DigitalOutput laserOutput = new DigitalOutput(4);
    PIDController aimPID = new PIDController(0.025, 0, 0);
    Rotation2d laserTargetAngle = new Rotation2d();

    public LaserSubsystem() {
        aimMotor.setInverted(false);

        aimEncoder.setPositionConversionFactor(1 / 3600);
        //aimEncoder.setVelocityConversionFactor(360 / (Constants.neoCountsPerRevolution * 10));
    }

    public void setLaserState(boolean on) {
        laserOutput.set(on);
    }

    public boolean getLaserState() {
        return laserOutput.get();
    }

    public void setAngle(Rotation2d angle) {
        laserTargetAngle = angle;
    }
  

    @Override
    public void periodic() {
        aimMotor.setVoltage(aimPID.calculate(aimEncoder.getPosition(), laserTargetAngle.getDegrees()));
        SmartDashboard.putNumber("getName(aimEncoder)", aimEncoder.getPosition());
        SmartDashboard.putNumber("getName(targetAngle)", laserTargetAngle.getDegrees());
    }
}
