package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class FieldCentric extends Command {
	private SwerveSubsystem driveSubsystem;

	public FieldCentric(SwerveSubsystem swerveSubsystem) {
		addRequirements(swerveSubsystem);
		driveSubsystem = swerveSubsystem;
	}

	@Override
	public void execute() {
		double xSpeed = MathUtil.applyDeadband(Constants.xSpeedLimiter.calculate(-Constants.translationJoystick.getY()), Constants.deadband);
		double ySpeed = MathUtil.applyDeadband(Constants.ySpeedLimiter.calculate(Constants.translationJoystick.getX()), Constants.deadband);
		double zRot = MathUtil.applyDeadband(Constants.zRotSpeedLimiter.calculate(Constants.rotationJoystick.getX()), Constants.deadband);

		double distance = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
		if(distance > 1) {
			xSpeed /= distance;
			ySpeed /= distance;
		}
		
		driveSubsystem.drive(
			xSpeed * Constants.maxSpeed,
			ySpeed * Constants.maxSpeed, 
			zRot * Constants.maxAngularSpeed,
			true);
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