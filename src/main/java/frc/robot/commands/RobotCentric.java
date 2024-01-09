package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
		driveSubsystem.driveRobotCentric(
			MathUtil.applyDeadband(Constants.translationJoystick.getY(), Constants.deadband),
			MathUtil.applyDeadband(Constants.translationJoystick.getX(), Constants.deadband),
			MathUtil.applyDeadband(-Constants.rotationJoystick.getX(), Constants.deadband));
	}

	@Override
	public void end(boolean interrupted) {
		driveSubsystem.driveRobotCentric(0, 0, 0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}