package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class FieldCentric extends CommandBase {
	private SwerveSubsystem driveSubsystem;

	public FieldCentric(SwerveSubsystem swerveSubsystem) {
		addRequirements(swerveSubsystem);
		driveSubsystem = swerveSubsystem;
	}

	@Override
	public void execute() {
		driveSubsystem.driveFieldCentric(
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