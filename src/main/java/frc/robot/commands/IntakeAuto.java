package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeAuto extends Command {
    ShooterSubsystem shooting;
    public IntakeAuto (ShooterSubsystem shooter) {
        shooting = shooter;
    }
    
    @Override
    public void execute () {
        shooting.setIntakeVoltage(12);
    }
}
