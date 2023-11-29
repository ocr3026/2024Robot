package frc.robot.util;

public class OcrMath {
	public static double clamp(double value, double min, double max) {
		return Math.max(min, Math.min(max, value));
	}

	public static boolean isWithin(double value, double range) {
		return value >= -range && value <= range;
	}

	public static double deadband(double value, double deadband) {
		if (value > deadband) {
			return (value - deadband) / (1 - deadband);
		} else if (value < -deadband) {
			return (value + deadband) / (1 - deadband);
		} else {
			return 0;
		}
	}
}