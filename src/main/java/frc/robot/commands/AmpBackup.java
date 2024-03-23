package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveOrigin;

public class AmpBackup extends Command {
    SwerveSubsystem swerveSubsystem;
    Translation2d initialTranslation;

    PIDController xPID = new PIDController(1, 0, 0);

    public AmpBackup(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        xPID.setTolerance(0.01);
    }

    @Override
    public void initialize() {
        initialTranslation = swerveSubsystem.getPose().getTranslation();
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(xPID.calculate(swerveSubsystem.getPose().getTranslation().getDistance(initialTranslation), 0.1016), 0, 0, DriveOrigin.RobotCentric);
    }

    @Override
    public boolean isFinished() {
        return xPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new ChassisSpeeds());
    }
}
