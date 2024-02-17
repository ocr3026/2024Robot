package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ServoCommand extends Command{

    ShooterSubsystem shooterSubsystem;

    @Override
    public void execute() {
        shooterSubsystem.setSpeed(Constants.xbox.getRightY());
    }
    
}
