package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ServoCommand extends Command{

    ShooterSubsystem shooterSubsystem;

    public ServoCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        
<<<<<<< Updated upstream
        shooterSubsystem.setActuatorPos(-Constants.xbox.getRightY());
=======
        shooterSubsystem.setSpeed(MathUtil.clamp(-Constants.xbox.getRightY(), 0, 1));
>>>>>>> Stashed changes
    }
    
}
