package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAuto extends Command {
    ShooterSubsystem shooterSubsystem;
    Timer timer = new Timer();
    public ShootAuto (ShooterSubsystem shooter) {
        shooterSubsystem = shooter;
    }

    @Override 
    public void initialize() {
        shooterSubsystem.setFlywheelSpeeds(0, 0);
        shooterSubsystem.setIntakeVoltage(0);
        timer.start();
        
    }
    @Override
    public void execute() {
        if (!timer.hasElapsed(1)) {
            shooterSubsystem.setFlywheelSpeeds(5660 * .8, 5660 * .85);
        }
        else {
            shooterSubsystem.setFlywheelSpeeds(0, 0);
        }
        if (timer.hasElapsed(1) && !timer.hasElapsed(1.2)) {
            shooterSubsystem.setIntakeVoltage(10);
        }
        else {
            shooterSubsystem.setIntakeVoltage(0);
        }
     }
     @Override
     public boolean isFinished() {
         // TODO Auto-generated method stub
         return timer.hasElapsed(1.5);
     }
     @Override
     public void end(boolean interrupted) {
         // TODO Auto-generated method stub
         shooterSubsystem.setFlywheelSpeeds(0, 0);
         shooterSubsystem.setIntakeVoltage(0);
     }
}
