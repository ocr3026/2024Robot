package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PulleySubsystem extends SubsystemBase {

    CANSparkMax aimMotor = new CANSparkMax(62, MotorType.kBrushless);


    public PulleySubsystem() {

    }

    public void setSpeed(double speed) {
        aimMotor.set(speed);
    }
    
}
