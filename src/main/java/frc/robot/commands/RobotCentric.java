package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotCentric extends Command {
	private SwerveSubsystem driveSubsystem;

	public RobotCentric(SwerveSubsystem swerveSubsystem) {
		addRequirements(swerveSubsystem);
		driveSubsystem = swerveSubsystem;
	}

	@Override
	public void execute() {
		double xSpeed = MathUtil.applyDeadband(Constants.xSpeedLimiter.calculate(Constants.translationJoystick.getY()), Constants.deadband);
		double ySpeed = MathUtil.applyDeadband(Constants.ySpeedLimiter.calculate(Constants.translationJoystick.getX()), Constants.deadband);
		double zRot = MathUtil.applyDeadband(Constants.zRotSpeedLimiter.calculate(-Constants.rotationJoystick.getX()), Constants.deadband);

		double distance = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
		if(distance > 1) {
			xSpeed /= distance;
			ySpeed /= distance;
		}

		// TODO: Make xSpeed be driven by xSpeed instead of 1

		driveSubsystem.drive(
			1 * Constants.maxSpeed,
			ySpeed * Constants.maxSpeed, 
			zRot * Constants.maxAngularSpeed,
			false);
	}

	@Override
	public void end(boolean interrupted) {
		driveSubsystem.drive(0, 0, 0, false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}