package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeAuto extends Command {
    ShooterSubsystem shooterSubsystem;
    Timer timer = new Timer();

    public IntakeAuto (ShooterSubsystem subsystem) {
        shooterSubsystem = subsystem;
    }

    @Override
    public void initialize () {
        shooterSubsystem.setIntakeVoltage(0);
        timer.reset();
        timer.start();
    }
    @Override
    public void execute() {
        if(!timer.hasElapsed(1)) {
            shooterSubsystem.setIntakeVoltage(12);
        }
        else {
            shooterSubsystem.setIntakeVoltage(0);
        }
    }

    @Override
    public boolean isFinished() {
        
        return timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        shooterSubsystem.setIntakeVoltage(0);
        timer.stop();
        timer.reset();
    }

}
