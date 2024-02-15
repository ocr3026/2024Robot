package frc.robot.keybinds.manipulators;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.keybinds.ManipulatorProfile;

public class Evan implements ManipulatorProfile {
    @Override
    public Trigger shootTrigger() {
        return Constants.xbox.rightTrigger();
    }

    @Override 
    public Trigger intakeTrigger() {
        return Constants.xbox.a();
    }

    @Override 
    public Trigger exhaustTrigger() {
        return Constants.xbox.b();
    }
}
