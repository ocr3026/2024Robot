package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command{
    ShooterSubsystem shooterSubsystem;

    public Shoot(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
<<<<<<< HEAD
        if(Constants.xbox.getLeftTriggerAxis() < 0.5) {
            shooterSubsystem.setFlywheelVoltage(12,11.5);

            if(Constants.xbox.getLeftY() < -0.5) {
                shooterSubsystem.setIntakeVoltage(4);
            } else {
                shooterSubsystem.setIntakeVoltage(0);
            }
        } else {
            shooterSubsystem.setFlywheelVoltage(5, 5);

            if(Constants.xbox.getLeftY() < -0.5) {
                shooterSubsystem.setIntakeVoltage(3.9);
            } else {
                shooterSubsystem.setIntakeVoltage(0);
            }
=======
        shooterSubsystem.setFlywheelVoltage(12,11.5);
        
        if(Constants.xbox.getLeftY() < -0.5) {
            shooterSubsystem.setIntakeVoltage(4);
        } else {
            shooterSubsystem.setIntakeVoltage(0);
>>>>>>> parent of e6efbf1 (tweaking shoot)
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelVoltage(0, 0);
        shooterSubsystem.setIntakeVoltage(0);
    }
}
