package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        shooterSubsystem.setFlywheelVoltage(0, 0);
        shooterSubsystem.setIntakeVoltage(0);
        timer.reset();
        timer.start();
        
    }
    @Override
    public void execute() {
        SmartDashboard.putNumber("Timer", timer.get());
        if (!timer.hasElapsed(1)) {
            shooterSubsystem.setFlywheelVoltage(12, 12);
        }
        else {
            shooterSubsystem.setFlywheelVoltage(0, 0);
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
         return timer.hasElapsed(1.5);
     }
     @Override
     public void end(boolean interrupted) {
         shooterSubsystem.setFlywheelVoltage(0, 0);
         shooterSubsystem.setIntakeVoltage(0);
         timer.stop();
         timer.reset();
     }
}
