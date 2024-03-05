package frc.robot.commands;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ServoToTarget extends Command {
    ShooterSubsystem shooterSubsystem;
    public ServoToTarget (ShooterSubsystem  shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        if(Constants.camera.isPresent()) {
            PhotonPipelineResult result = Constants.camera.get().getLatestResult();
            PhotonTrackedTarget target = result.getBestTarget();

            if(target != null) {
                double dist = target.getBestCameraToTarget().getX();

                if(target.getFiducialId() == 5 || target.getFiducialId() == 6) {
                    shooterSubsystem.setActuatorPos(1);
                }

                if(target.getFiducialId() == 4 || target.getFiducialId() == 7) {
                    shooterSubsystem.setActuatorPos(Constants.a * Math.pow(dist, 3) + Constants.b * Math.pow(dist, 2) + Constants.c * dist + Constants.d);
                }
            }
        }
    }

}
