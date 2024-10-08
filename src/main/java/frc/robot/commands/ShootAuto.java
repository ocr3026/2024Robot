package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAuto extends Command {
    boolean stop = false;
    ShooterSubsystem shooterSubsystem;
    Timer timer = new Timer();
    public ShootAuto (ShooterSubsystem shooter) {
        shooterSubsystem = shooter;
    }

    @Override 
    public void initialize() {
        shooterSubsystem.setFlywheelVoltage(0, 0);
        shooterSubsystem.setIntakeVoltage(0);
        timer.reset();
        timer.start();
        stop = false;
        
    }
    @Override
    public void execute() {
        SmartDashboard.putNumber("Timer", timer.get());
        if (!timer.hasElapsed(.1)) {
            shooterSubsystem.setIntakeVoltage(-9);
            shooterSubsystem.setFlywheelVoltage(12, 11);
        }
        if (!timer.hasElapsed(.9)) {
            shooterSubsystem.setFlywheelVoltage(12, 11);

        }
        if (timer.hasElapsed(.1)) {
            stop = true;
        }
        if (stop) {
            shooterSubsystem.setIntakeVoltage(0);
        }
  
        else {
            shooterSubsystem.setFlywheelVoltage(0, 0);
        }
        if (timer.hasElapsed(.55) && !timer.hasElapsed( .75  )) {
            stop = false;
            shooterSubsystem.setIntakeVoltage(12);
            
        }
     }
     @Override
     public boolean isFinished() {
         return timer.hasElapsed(.75);
     }
     @Override
     public void end(boolean interrupted) {
        stop = false;
         shooterSubsystem.setFlywheelSpeed(0, 0);
         shooterSubsystem.setIntakeVoltage(0);
         timer.stop();
         timer.reset();
     }
}
