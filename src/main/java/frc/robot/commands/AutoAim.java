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

    double servoPos = 0;

    public AutoAim(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
        xPID.setTolerance(0.05);
        yPID.setTolerance(0.05);
    }
   // 77 inches from the speaker  shootin !?

    @Override
    public void execute() {
        PhotonTrackedTarget target = Constants.camera.get().getLatestResult().getBestTarget();

        if(target != null) {
            int tagID = target.getFiducialId();
            double yaw = target.getYaw();
            double zAngle = Math.toDegrees(target.getBestCameraToTarget().getRotation().getAngle());
            Transform3d camToTarget = target.getBestCameraToTarget();
            double yDist = camToTarget.getY();
            
            if(tagID == 4 || tagID == 7) {
                swerveSubsystem.drive(0, 0, -MathUtil.clamp(rotatePID.calculate(yaw, 0), -0.3, 0.3), DriveOrigin.RobotCentric);
                
                if(yDist >= 1 && yDist < 2) {
                    servoPos = Constants.autoServoToSpeaker.get(1) - (((1 - yDist) / (1 - 2)) * (Constants.autoServoToSpeaker.get(1) - Constants.autoServoToSpeaker.get(2)));
                    shooterSubsystem.setActuatorPos(servoPos);
                }
                else if(yDist >= 2 && yDist < 3) {
                    servoPos = Constants.autoServoToSpeaker.get(2) - (((2 - yDist) / (1 - 2)) * (Constants.autoServoToSpeaker.get(2) - Constants.autoServoToSpeaker.get(3)));
                    shooterSubsystem.setActuatorPos(servoPos);

                }
                else if(yDist >= 3 && yDist < 4) {
                    servoPos = Constants.autoServoToSpeaker.get(3) - (((3 - yDist) / (1 - 2)) * (Constants.autoServoToSpeaker.get(3) - Constants.autoServoToSpeaker.get(4)));
                    shooterSubsystem.setActuatorPos(servoPos);

                }
                else if(yDist >= 4 && yDist < 5) {
                    servoPos = Constants.autoServoToSpeaker.get(4) - (((4 - yDist) / (1 - 2)) * (Constants.autoServoToSpeaker.get(4) - Constants.autoServoToSpeaker.get(5)));
                    shooterSubsystem.setActuatorPos(servoPos);

                }

                if(-2 <= yaw && 2 >= yaw) {
                    swerveSubsystem.drive(0, 0, 0, DriveOrigin.RobotCentric);
                }
            }

            if(tagID == 5 || tagID == 6) {
                swerveSubsystem.drive(0, 0, -MathUtil.clamp(rotatePID.calculate(zAngle, 180), -0.3, 0.3), DriveOrigin.RobotCentric);

                if(yDist >= 1 && yDist < 2) {
                    servoPos = Constants.autoServoToAmp.get(1) - (((1 - yDist) / (1 - 2)) * (Constants.autoServoToAmp.get(1) - Constants.autoServoToAmp.get(2)));
                                        shooterSubsystem.setActuatorPos(servoPos);

                }
                else if(yDist >= 2 && yDist < 3) {
                    servoPos = Constants.autoServoToAmp.get(2) - (((2 - yDist) / (1 - 2)) * (Constants.autoServoToAmp.get(2) - Constants.autoServoToAmp.get(3)));
                    shooterSubsystem.setActuatorPos(servoPos);

                }
                else if(yDist >= 3 && yDist < 4) {
                    servoPos = Constants.autoServoToAmp.get(3) - (((3 - yDist) / (1 - 2)) * (Constants.autoServoToAmp.get(3) - Constants.autoServoToAmp.get(4)));
                    shooterSubsystem.setActuatorPos(servoPos);

                }
                else if(yDist >= 4 && yDist < 5) {
                    servoPos = Constants.autoServoToAmp.get(4) - (((4 - yDist) / (1 - 2)) * (Constants.autoServoToAmp.get(4) - Constants.autoServoToAmp.get(5)));
                                        shooterSubsystem.setActuatorPos(servoPos);

                }

                for(int x = 1; yDist >= x && yDist < (x+1); x++) {

                }
                
                if(-2 <= yaw && 2 >= yaw) {
                    swerveSubsystem.drive(0, 0, 0, DriveOrigin.RobotCentric);
                }

                if(zAngle <= 184 && zAngle >= 176) {
                    swerveSubsystem.drive(xPID.calculate(camToTarget.getX(), 0.8382), yPID.calculate(camToTarget.getY(), 0), 0, DriveOrigin.RobotCentric);
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
        shooterSubsystem.setActuatorPos(0);

    }
    
    
}
