package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
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


    public AutoAim(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(swerveSubsystem, shooterSubsystem);
        xPID.setTolerance(0.05);
        yPID.setTolerance(0.05);
        rotatePID.setTolerance(0.05);
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = Constants.camera.get().getLatestResult().getBestTarget();

        if(target != null) {
            int tagID = target.getFiducialId();
            double yaw = target.getYaw();
            double zAngle = Math.toDegrees(target.getBestCameraToTarget().getRotation().getAngle());
            Transform3d camToTarget = target.getBestCameraToTarget();
            
            if(tagID == 4 || tagID == 7) {
                swerveSubsystem.drive(0, 0, -MathUtil.clamp(rotatePID.calculate(yaw, 0), -0.3, 0.3), DriveOrigin.RobotCentric);
            
                if(rotatePID.atSetpoint()) {
                    double dist = camToTarget.getY();
                    shooterSubsystem.setActuatorPos(Constants.a * Math.pow(dist, 3) + Constants.b * Math.pow(dist, 2) + Constants.c * dist + Constants.d);
                }
            }

            if(tagID == 5 || tagID == 6) {
                swerveSubsystem.drive(0, 0, -MathUtil.clamp(rotatePID.calculate(zAngle, 180), -0.3, 0.3), DriveOrigin.RobotCentric);

                if(rotatePID.atSetpoint()) {
                    swerveSubsystem.drive(xPID.calculate(camToTarget.getX(), 0.8382), yPID.calculate(camToTarget.getY(), 0), 0, DriveOrigin.RobotCentric);
                    
                    if(xPID.atSetpoint() && yPID.atSetpoint()) {
                        shooterSubsystem.setActuatorPos(1);
                    }
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
    
}