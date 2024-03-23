package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveOrigin;

public class AutoAimAndShoot extends Command {
    SwerveSubsystem swerveSubsystem;
    ShooterSubsystem shooterSubsystem;

    PIDController rotatePID = new PIDController(0.1, 0, 0);
    PIDController xPID = new PIDController(0.1, 0, 0);
    PIDController yPID = new PIDController(0.1, 0, 0);

    boolean isFinished = false;
    boolean inPosition = false;
    Timer timer = new Timer();

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
        timer.reset();
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

                if(rotatePID.atSetpoint()) {
                    double dist = camToTarget.getX();
                    shooterSubsystem.setCamPos(calculateCamPos(dist));
                    swerveSubsystem.drive(0, 0, 0, DriveOrigin.RobotCentric);
                    inPosition = true;
                    if (inPosition) {
                        shooterSubsystem.setFlywheelSpeed(1, 1);
                        if (shooterSubsystem.getFlywheelSpeed() > .99) {
                            if(timer.hasElapsed(.4)) {
                                shooterSubsystem.setIntakeVoltage(10);
                            }
                            if (timer.hasElapsed(.8)) {
                                isFinished = true;
                            }
                        }
                        else {
                            timer.reset();
                        }
                    }
                }
                else if (!rotatePID.atSetpoint()){
                    swerveSubsystem.drive(0, 0, rotatePID.calculate(yaw, 0), DriveOrigin.RobotCentric);
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(new ChassisSpeeds());
        shooterSubsystem.setFlywheelSpeed(0,0);
        shooterSubsystem.setIntakeVoltage(0);
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