package frc.robot.keybinds.manipulators;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.keybinds.ManipulatorProfile;

public class Evan implements ManipulatorProfile {
    @Override 
    public Trigger windUpTrigger() {
        // TODO Auto-generated method stub
        return Constants.xbox.a();
    }
    @Override
    public Trigger unwindTrigger() {
        return Constants.xbox.b();
    }
}
