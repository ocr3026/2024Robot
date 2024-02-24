package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveTo extends Command {
    SwerveSubsystem swerveSubsystem;

    Pose2d targetPose;
    PIDController xPidController = new PIDController(0.25, 0, 0);
    PIDController yPidController = new PIDController(0.25, 0, 0);
    PIDController rotPidController = new PIDController(0.1, 0, 0);

    public DriveTo(SwerveSubsystem swerveSubsystem, Pose2d target) {
        this.swerveSubsystem = swerveSubsystem;
        targetPose = target;
        xPidController.setTolerance(0.05);
        yPidController.setTolerance(0.05);
        rotPidController.setTolerance(0.05);
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(MathUtil.clamp(xPidController.calculate(swerveSubsystem.getPose().getX(), targetPose.getX()), -0.4, 0.4),
                              MathUtil.clamp(yPidController.calculate(swerveSubsystem.getPose().getY(), targetPose.getY()), -0.4, 0.4), 
                              MathUtil.clamp(rotPidController.calculate(swerveSubsystem.getPose().getRotation().getRadians(), targetPose.getRotation().getRadians()), -0.4, 0.4), true);
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
