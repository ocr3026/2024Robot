package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;


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
        climber.climbSpeedLeft(-Constants.xbox.getRightY());
        climber.climbSpeedRight(Constants.xbox.getLeftY());
        // free dizzo free j haus "chat is this freelo?" - jaimon - "call me bloonarius the way my kids shot across the screen" - jaimon - "they say opposites attract - thats why money comes slow and goes fast... and i go slow and come fast" - j money
    }

    @Override
    public void end(boolean interupt) {
        climber.climbSpeed(0);
    }
}
