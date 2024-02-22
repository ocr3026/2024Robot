package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /*   PIDController PID = new PIDController(0, 0, 0);
    CANSparkMax leftClimb = new  CANSparkMax(31, MotorType.kBrushless);
    CANSparkMax rightClimb = new CANSparkMax(32, MotorType.kBrushless);

    public ClimberSubsystem () {
        leftClimb.getEncoder().setPositionConversionFactor(1);

        leftClimb.setIdleMode(IdleMode.kBrake);
        rightClimb.setIdleMode(IdleMode.kBrake);
    }
     public void climbSpeed (double speed) {
        leftClimb.set(speed);
        rightClimb.set(speed);
     }*/

}
