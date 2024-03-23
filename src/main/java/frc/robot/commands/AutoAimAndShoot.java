package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveOrigin;

public class AutoAimAndShoot extends Command {
    SwerveSubsystem swerveSubsystem;
    ShooterSubsystem shooterSubsystem;

    boolean inPosition = false;

    PIDController rotatePID = new PIDController(0.1, 0, 0);
    PIDController xPID = new PIDController(0.1, 0, 0);
    PIDController yPID = new PIDController(0.1, 0, 0);
    Timer timer = new Timer();

    boolean isFinished = false;

    public AutoAimAndShoot(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem) {
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
        inPosition = false;
        timer.start();
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

        if(target == null)
            swerveSubsystem.drive(0, 0, 0, DriveOrigin.RobotCentric);

        if(target != null) {
            int tagID = target.getFiducialId();
            double yaw = target.getYaw();
            Transform3d camToTarget = target.getBestCameraToTarget();
            
            if(tagID == 4 || tagID == 7) {
                swerveSubsystem.drive(0, 0, -rotatePID.calculate(yaw, 0), DriveOrigin.RobotCentric);

                if(rotatePID.atSetpoint()) {
                    double dist = camToTarget.getX();
                    shooterSubsystem.setCamPos(calculateCamPos(dist));
                    swerveSubsystem.drive(0, 0, 0, DriveOrigin.RobotCentric);
                    inPosition = true;
                }
                if(inPosition  ) {
                    shooterSubsystem.setFlywheelVoltage(12, 12);
                    if (shooterSubsystem.getFlywheelSpeed() > .95) {
                        shooterSubsystem.setIntakeVoltage(8);
                        if (timer.hasElapsed(.1)) {
                            isFinished = true;
                        }
                    }
                    else {
                        timer.reset();
                    }

                    
                }
            }
            
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new ChassisSpeeds());
        shooterSubsystem.setFlywheelVoltage(0, 0);
        shooterSubsystem.setIntakeVoltage(0);
        timer.stop();
        timer.reset();
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