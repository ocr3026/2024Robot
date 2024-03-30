package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveOrigin;

public class ZeroYaw extends Command{
    boolean isFinished;
    SwerveSubsystem swerveSubsystem;
    PIDController rotatePID = new PIDController(0.06, 0, 0);
    public ZeroYaw(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        rotatePID.setTolerance(2);
    }
    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(0,0,rotatePID.calculate(-swerveSubsystem.getPose().getRotation().getDegrees() , 0) , DriveOrigin.RobotCentric);
        if(rotatePID.atSetpoint())
            isFinished = true;
    }
    @Override
    public boolean isFinished() {
        return isFinished;
    }
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0, 0, 0, DriveOrigin.RobotCentric);
    }
    
}
