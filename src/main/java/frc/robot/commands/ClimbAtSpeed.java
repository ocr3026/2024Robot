package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class ClimbAtSpeed extends Command {
    ClimberSubsystem climber;
    double speed;
    
    public ClimbAtSpeed ( ClimberSubsystem climberSubsystem) {
        climber = climberSubsystem;
    }

    @Override
    public void initialize () {
        climber.climbSpeed(0);
    }

    @Override
    public void execute () {
        climber.climbSpeedLeft(Constants.xbox.getLeftY());
        climber.climbSpeedRight(Constants.xbox.getRightY());
    }

    @Override
    public void end(boolean interupt) {
        climber.climbSpeed(0);
    }
}
