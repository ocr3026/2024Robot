package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
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
		Translation2d translation = new Translation2d(
			MathUtil.applyDeadband(Constants.translationJoystick.getY(), Constants.deadband), 
			MathUtil.applyDeadband(Constants.translationJoystick.getX(), Constants.deadband));

		if(translation.getNorm() > Constants.maxSpeed) {
			translation = translation.div(translation.getNorm());
		}

		driveSubsystem.drive(
			translation.getX(),
			translation.getY(), 
			MathUtil.applyDeadband(-Constants.rotationJoystick.getX(), Constants.deadband),
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