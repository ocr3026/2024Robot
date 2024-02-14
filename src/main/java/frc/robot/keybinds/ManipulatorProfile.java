package frc.robot.keybinds;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ManipulatorProfile {
    public Trigger windUpTrigger();
    public Trigger unwindTrigger();
}
