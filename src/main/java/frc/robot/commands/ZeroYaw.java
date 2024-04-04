package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveOrigin;

public class ZeroYaw extends Command{
    boolean isFinished;
    SwerveSubsystem swerveSubsystem;
    PIDController rotatePID = new PIDController(0.05, 0, 0);
    public ZeroYaw(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        rotatePID.setTolerance(1);
        rotatePID.enableContinuousInput(-180, 180);
    }
    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            swerveSubsystem.drive(0,0,rotatePID.calculate(-swerveSubsystem.getPose().getRotation().getDegrees() , 180) , DriveOrigin.RobotCentric);
            if(rotatePID.atSetpoint()) {
                isFinished = true;
            }
        }
        else {
            swerveSubsystem.drive(0,0,rotatePID.calculate(-swerveSubsystem.getPose().getRotation().getDegrees() , 0) , DriveOrigin.RobotCentric);
            if(rotatePID.atSetpoint()) {
                isFinished = true;
            }
        }   
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
