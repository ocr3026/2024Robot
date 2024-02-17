package frc.robot.commands;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LaserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Laser extends Command {
    LaserSubsystem laserSubsystem;
    SwerveSubsystem swerveSubsystem;

    PIDController yawPID = new PIDController(0.001, 0, 0.00001);
    PIDController yPID = new PIDController(1.4, 0, 0);
    


    public Laser(LaserSubsystem laserSubsystem, SwerveSubsystem swerveSubsystem) {
        this.laserSubsystem = laserSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(laserSubsystem, swerveSubsystem);
    }

    PhotonTrackedTarget target;
    SendableChooser<PhotonTrackedTarget> sendableChooser = new SendableChooser<>();


    @Override
    public void initialize() {
        target = null;
        sendableChooser.addOption("none", null);
    }

    @Override
    public void execute() {
        
        PhotonPipelineResult cameraResult = Constants.camera.getLatestResult();
        List<PhotonTrackedTarget> listTargets = cameraResult.getTargets();


        for(PhotonTrackedTarget i : listTargets) {
            sendableChooser.addOption(Integer.toString(i.getFiducialId()), i);
        }
        
        target = sendableChooser.getSelected();
        
        SmartDashboard.putData("SENDABLE CHOOSER POR QUE", sendableChooser);
        SmartDashboard.updateValues();


        if(target != null) {
            if(sendableChooser.getSelected() != null) {
            SmartDashboard.putString("Current Target", Integer.toString(sendableChooser.getSelected().getFiducialId()));
            }
            else {
                SmartDashboard.putString("Current Target", "NULL");
            }

            double distY = target.getBestCameraToTarget().getY();
            double yaw = Math.toDegrees(target.getBestCameraToTarget().getRotation().getAngle());
    


            //swerveSubsystem.drive(0, 0, yawPID.calculate(yaw, 0), false);

            swerveSubsystem.drive(0,0, MathUtil.clamp(yawPID.calculate(yaw, 180), -0.2, 0.2), false);
          
              if(yaw <= 190 && yaw >= 170 ) {
                swerveSubsystem.drive(0, yPID.calculate(distY, 0),0 ,false);

            }

            

         
            SmartDashboard.putNumber("Rotation Z", yaw);

            //laserSubsystem.setAngle(Rotation2d.fromRadians(-Math.atan((inchesVertical) / (inchesDepth))));
            //laserSubsystem.setAngle(Rotation2d.fromRadians(2 * Math.PI));


        } else {
            laserSubsystem.setLaserState(false);
            swerveSubsystem.drive(0, 0, 0, false);
            target = null;
        }
    }

    @Override
    public void end(boolean interrupted) {
        laserSubsystem.setLaserState(false);
        swerveSubsystem.drive(0, 0, 0, false);
        target = null;
    }
}
