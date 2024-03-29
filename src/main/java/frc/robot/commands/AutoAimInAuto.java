package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


public class AutoAimInAuto extends Command {
    ShooterSubsystem shooterSubsystem;

    PIDController xPID = new PIDController(0.1, 0, 0);
    PIDController yPID = new PIDController(0.1, 0, 0);

    boolean isFinished = false;

    public AutoAimInAuto(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements( shooterSubsystem);
        xPID.setTolerance(0.05);
        yPID.setTolerance(0.05);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = null;

        PhotonCamera camera = Constants.camera.get();
        for(PhotonTrackedTarget i : camera.getLatestResult().targets) {
            if(i.getFiducialId() == 7 || i.getFiducialId() == 4) {
                target = i;
                break;
            }
        }


        if(target != null) {
            int tagID = target.getFiducialId();
            Transform3d camToTarget = target.getBestCameraToTarget();
            
            if(tagID == 4 || tagID == 7) {

                    double dist = camToTarget.getX();
                    shooterSubsystem.setCamPos(calculateCamPos(dist));
                    isFinished = true;
                
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
    
    public static double calculateCamPos(double dist) {
        SmartDashboard.putNumber("curveInput", dist);
        double out = (Constants.a * Math.pow(dist, 3)) + (Constants.b * Math.pow(dist, 2)) + (Constants.c * dist) + Constants.d;
        SmartDashboard.putNumber("curveOutput", out);
        return out;
    }
}