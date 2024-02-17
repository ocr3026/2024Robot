package frc.robot.commands;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LaserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Center extends Command {
    LaserSubsystem laserSubsystem;
    SwerveSubsystem swerveSubsystem;
    Transform3d fieldToCamera;
     PIDController yawPID = new PIDController(0.001, 0, 0.00001);
    public Center (LaserSubsystem LaserSubsystem, SwerveSubsystem SwerveSubsystem ) {
        laserSubsystem = LaserSubsystem;
        swerveSubsystem = SwerveSubsystem;
        
    }

    PhotonTrackedTarget target;

    @Override
    public void initialize() {
       
        target= null;
    
    }

    @Override
    public void execute () {
        PhotonPipelineResult cameraResult = Constants.camera.getLatestResult();
        if (cameraResult.getMultiTagResult().estimatedPose.isPresent) {
            fieldToCamera = cameraResult.getMultiTagResult().estimatedPose.best;
        }       
        
    if(fieldToCamera.getRotation().toRotation2d().getDegrees() >50 || fieldToCamera.getRotation().toRotation2d().getDegrees() < 40) {
        yawPID.calculate(fieldToCamera.getRotation().toRotation2d().getDegrees(), 0);
    }
}
}
