package frc.robot.keybinds.drivers;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.keybinds.DriverProfile;

public class Tatum implements DriverProfile {
	@Override
	public Trigger halfSpeedTrigger() {
		return Constants.translationJoystick.button(1);
	}

	@Override
	public Trigger zeroGyroTrigger() {
		return Constants.translationJoystick.button(12);
	}
}
