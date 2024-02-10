package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LaserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Laser extends Command {
    LaserSubsystem laserSubsystem;
    SwerveSubsystem swerveSubsystem;

    PIDController yawPID = new PIDController(0.05, 0, 0);
    PIDController yPID = new PIDController(1.4, 0, 0);
    


    public Laser(LaserSubsystem laserSubsystem, SwerveSubsystem swerveSubsystem) {
        this.laserSubsystem = laserSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(laserSubsystem, swerveSubsystem);
    }

    @Override
    public void execute() {
        
        PhotonPipelineResult cameraResult = Constants.camera.getLatestResult();
        PhotonTrackedTarget target = cameraResult.getBestTarget();

        if(target != null) {
            double distY = target.getBestCameraToTarget().getY();
            double yaw = Math.toDegrees(target.getBestCameraToTarget().getRotation().getAngle());
            Translation3d laserToTarget = target.getBestCameraToTarget().getTranslation().minus(Constants.laserToCamera);
            /*double yaw = PhotonUtils.getYawToPose(new Pose2d(), 
                    new Pose2d(laserToTarget.getX(),
                        laserToTarget.getY(), new Rotation2d())).getDegrees();

            /*swerveSubsystem.drive(0, 0, 
                yawPID.calculate(PhotonUtils.getYawToPose(new Pose2d(), 
                    new Pose2d(laserToTarget.getX(),
                        laserToTarget.getY(), new Rotation2d())).getDegrees(),
                    0), false);*/

            //swerveSubsystem.drive(0, 0, yawPID.calculate(yaw, 0), false);

                swerveSubsystem.drive(0,0, -MathUtil.clamp(yawPID.calculate(yaw, 180), -0.4, 0.4), false);
            
            
            //if(yaw == 0) {
                

            //}
          
              if(yaw <= 182 && yaw >= 178 ) {
                swerveSubsystem.drive(0, yPID.calculate(distY, 0),0 ,false);

                if(distY <= 0.02 && distY >= -0.02) {
                    swerveSubsystem.drive(0, 0, 0, false);
                    laserSubsystem.setLaserState(true);
                }
            }

            

            laserSubsystem.setAngle(Rotation2d.fromRadians(-Math.atan(laserToTarget.getZ() / laserToTarget.getX())));
            SmartDashboard.putNumber("translation3d getZ", laserToTarget.getZ());
            SmartDashboard.putNumber("translation3d getX", laserToTarget.getX());
            SmartDashboard.putNumber("Rotation Z", yaw);

            //laserSubsystem.setAngle(Rotation2d.fromRadians(-Math.atan((inchesVertical) / (inchesDepth))));
            //laserSubsystem.setAngle(Rotation2d.fromRadians(2 * Math.PI));


        } else {
            laserSubsystem.setLaserState(false);
            swerveSubsystem.drive(0, 0, 0, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        laserSubsystem.setLaserState(false);
        swerveSubsystem.drive(0, 0, 0, false);
    }
}
