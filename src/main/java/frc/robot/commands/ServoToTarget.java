package frc.robot.commands;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ServoToTarget extends Command {
    ShooterSubsystem shooterSubsystem;
    SwerveSubsystem swerveSubsystem;
    public ServoToTarget (ShooterSubsystem  shooterSubsystem, SwerveSubsystem swerveSubsystem) {
        this.shooterSubsystem =shooterSubsystem;
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void execute() {
       PhotonPipelineResult result = Constants.camera.getLatestResult();
       PhotonTrackedTarget target = result.getBestTarget();
       target.getBestCameraToTarget(). getX();
    }

}
