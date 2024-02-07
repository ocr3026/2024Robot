package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LaserSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.OcrMath;

public class Laser extends Command {
    LaserSubsystem laserSubsystem;
    SwerveSubsystem swerveSubsystem;

    PIDController yawPID = new PIDController(0, 0, 0);
    PIDController xPID = new PIDController(0, 0, 0);


    public Laser(LaserSubsystem laserSubsystem, SwerveSubsystem swerveSubsystem) {
        this.laserSubsystem = laserSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(laserSubsystem, swerveSubsystem);
    }

    @Override
    public void execute() {
        
       // PhotonPipelineResult cameraResult = Constants.camera.getLatestResult();
        //PhotonTrackedTarget target = cameraResult.getBestTarget();

        //if(target != null) {
            /*double distY = target.getBestCameraToTarget().getY();
            //double yaw = target.getBestCameraToTarget().getRotation().getAngle();
            Translation3d laserToTarget = target.getBestCameraToTarget().getTranslation().minus(Constants.laserToCamera);
            double yaw = PhotonUtils.getYawToPose(new Pose2d(), 
                    new Pose2d(laserToTarget.getX(),
                        laserToTarget.getY(), new Rotation2d())).getDegrees();*/

            /*swerveSubsystem.drive(0, 0, 
                yawPID.calculate(PhotonUtils.getYawToPose(new Pose2d(), 
                    new Pose2d(laserToTarget.getX(),
                        laserToTarget.getY(), new Rotation2d())).getDegrees(),
                    0), false);*/

            //swerveSubsystem.drive(0, 0, yawPID.calculate(yaw, 0), false);

            /*if(target.getYaw() != 0) {
                swerveSubsystem.drive(0,0, OcrMath.clamp(yawPID.calculate(yaw, 0), -0.2, 0.2), false);
            }*/
            
            //if(yaw == 0) {
                
                //swerveSubsystem.drive(OcrMath.clamp(xPID.calculate(distY, 0), -0.2, 0.2), 0 ,0 ,false);

            //}
             //if(distX == 0 ){
                swerveSubsystem.drive(0, 0, 0, false);
                laserSubsystem.setLaserState(true);
           // }


            float inchesVertical = 25;
            float inchesDepth = 25;


            //laserSubsystem.setAngle(Rotation2d.fromRadians(Math.atan(laserToTarget.getZ() / laserToTarget.getX())));
            SmartDashboard.putString("Laser:", "Rotating Laser");
            laserSubsystem.setAngle(Rotation2d.fromRadians(-Math.atan((inchesVertical / 39.37) / (inchesDepth / 39.37))));
           // laserSubsystem.setAngle(Rotation2d.fromRadians(2 * Math.PI));


        //} else {
            //laserSubsystem.setLaserState(false);
            //swerveSubsystem.drive(0, 0, 0, false);
        }
   // }

    @Override
    public void end(boolean interrupted) {
        laserSubsystem.setLaserState(false);
        swerveSubsystem.drive(0, 0, 0, false);
    }
}
