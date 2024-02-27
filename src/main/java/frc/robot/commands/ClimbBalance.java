package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ClimbBalance extends Command {
    ClimberSubsystem climberSubsystem;
    SwerveSubsystem swerveSubsystem;
    PIDController rightPID = new PIDController(0, 0, 0);
    PIDController leftPID = new PIDController(0, 0, 0);

    public ClimbBalance(ClimberSubsystem climberSubsystem, SwerveSubsystem swerveSubsystem) {
        this.climberSubsystem = climberSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(climberSubsystem, swerveSubsystem);
    }

    @Override
    public void execute() {
       if(swerveSubsystem.getGyroRoll().getDegrees() >= 2) {
            climberSubsystem.climbSpeedRight(0.25);
       }
     

       else if(swerveSubsystem.getGyroRoll().getDegrees() <= -2) {
        climberSubsystem.climbSpeedLeft(-0.25);
       }

       else {
        climberSubsystem.climbSpeed(0);
       }
       
        
    }
    @Override
    public void end(boolean interrupted) {
        climberSubsystem.climbSpeed(0);
    }
    
}
