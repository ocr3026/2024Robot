package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveTo extends Command {
    SwerveSubsystem swerveSubsystem;

    Pose2d targetPose;
    PIDController xPidController = new PIDController(0, 0, 0);
    PIDController yPidController = new PIDController(0, 0, 0);
    PIDController rotPidController = new PIDController(0, 0, 0);

    public DriveTo(SwerveSubsystem swerveSubsystem, Pose2d target) {
        this.swerveSubsystem = swerveSubsystem;
        targetPose = target;
        xPidController.setTolerance(0.01);
        yPidController.setTolerance(0.01);
        rotPidController.setTolerance(0.01);
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(xPidController.calculate(swerveSubsystem.getPose().getX(), targetPose.getX()),
                              yPidController.calculate(swerveSubsystem.getPose().getY(), targetPose.getY()), 
                              rotPidController.calculate(swerveSubsystem.getPose().getRotation().getRotations(), targetPose.getRotation().getRotations()), true);
    }

    @Override
    public boolean isFinished() {
        return xPidController.atSetpoint() && yPidController.atSetpoint() && rotPidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new ChassisSpeeds());
    }
}
