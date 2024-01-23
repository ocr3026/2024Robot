package frc.robot.keybinds;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverProfile {
	/**
	 * @return The trigger for toggling between field centric and robot centric while driving
	 */
	public Trigger halfSpeedTrigger();
}
