package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveOrigin;

public class AutoAim extends Command {
    SwerveSubsystem swerveSubsystem;
    ShooterSubsystem shooterSubsystem;

    PIDController rotatePID = new PIDController(0.1, 0, 0);
    PIDController xPID = new PIDController(0.1, 0, 0);
    PIDController yPID = new PIDController(0.1, 0, 0);

    boolean isFinished = false;

    public AutoAim(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(swerveSubsystem, shooterSubsystem);
        xPID.setTolerance(0.05);
        yPID.setTolerance(0.05);
        rotatePID.setTolerance(2);
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
            double yaw = target.getYaw();
            Transform3d camToTarget = target.getBestCameraToTarget();
            
            if(tagID == 4 || tagID == 7) {
                swerveSubsystem.drive(0, 0, -rotatePID.calculate(yaw, 0), DriveOrigin.RobotCentric);

                if(rotatePID.atSetpoint()) {
                    double dist = camToTarget.getX();
                    double camTarget = (Constants.a * Math.pow(dist, 3)) + (Constants.b * Math.pow(dist, 2)) + (Constants.c * dist) + Constants.d;
                    shooterSubsystem.setCamPos(camTarget);
                    swerveSubsystem.drive(0, 0, 0, DriveOrigin.RobotCentric);
                    isFinished = true;
                }
            }
        }
        else {
            swerveSubsystem.drive(new ChassisSpeeds());
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
    
}