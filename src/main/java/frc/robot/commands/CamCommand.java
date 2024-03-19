package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class CamCommand extends Command{

    ShooterSubsystem shooterSubsystem;

    public CamCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setCamPos((-Constants.xbox.getRightY() + 1) / 2);
        //shooterSubsystem.setCamDegrees(-Constants.xbox.getRightY());
    }
}
