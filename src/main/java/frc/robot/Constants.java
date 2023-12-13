package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.util.AHRSMixin;

public final class Constants {
	// Gyro
	public static final AHRSMixin gyro = new AHRSMixin(SerialPort.Port.kMXP);

	// Swerve Module Positions
	public static final Translation2d frontLeftModulePos = new Translation2d(-14.25, -14.25);
	public static final Translation2d rearLeftModulePos = new Translation2d(-14.25, 14.25);
	public static final Translation2d frontRightModulePos = new Translation2d(14.25, -14.25);
	public static final Translation2d rearRightModulePos = new Translation2d(14.25, 14.25);

	// Deadband
	public static final double deadband = 0.1;

	// Controls
	public static final CommandJoystick translationJoystick = new CommandJoystick(0);
	public static final CommandJoystick rotationJoystick = new CommandJoystick(1);
}
