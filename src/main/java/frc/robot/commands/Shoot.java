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
        if(Constants.xbox.getLeftTriggerAxis() < 0.5) {
            shooterSubsystem.setFlywheelVoltage(12,11.5);

            if(Constants.xbox.x().getAsBoolean()) {
                shooterSubsystem.setIntakeVoltage(4);
            } else {
                shooterSubsystem.setIntakeVoltage(0);
            }
        } else {
            shooterSubsystem.setFlywheelVoltage(5, 5);

            if(Constants.xbox.x().getAsBoolean()) {
                shooterSubsystem.setIntakeVoltage(4);
            } else {
                shooterSubsystem.setIntakeVoltage(0);
            }
        }

        
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelVoltage(0, 0);
        shooterSubsystem.setIntakeVoltage(0);
    }
}
