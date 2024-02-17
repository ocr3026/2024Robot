package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command{
    ShooterSubsystem shooterSubsystem;

    public Shoot(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setFlywheelSpeeds(5660 * .8, 5660 * .8);
        
        if(Constants.xbox.getLeftY() > .5) {
            shooterSubsystem.setIntakeVoltage(12);
        } else {
            shooterSubsystem.setIntakeVoltage(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelSpeeds(0, 0);
        shooterSubsystem.setIntakeVoltage(0);
    }
}
