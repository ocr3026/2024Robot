package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class CamCommandAuto extends Command{

    ShooterSubsystem shooterSubsystem;
    double camPos;
    boolean isFinished;

    public CamCommandAuto(ShooterSubsystem shooterSubsystem, double camPos) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);

    }

    @Override
    public void initialize() {
        isFinished = false;
    }
    @Override
    public void execute() {
        shooterSubsystem.setCamPos(ShooterSubsystem.camUpperLimit-.01);
        //shooterSubsystem.setCamDegrees(-Constants.xbox.getRightY());
        isFinished = true;
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return isFinished;
    }
}
