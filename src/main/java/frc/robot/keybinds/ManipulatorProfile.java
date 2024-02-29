package frc.robot.keybinds;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ManipulatorProfile {
    /**
     * @return The trigger for shooting
     */
    public Trigger shootTrigger();

    /**
     * @return The trigger for intaking
     */
    public Trigger intakeTrigger();

    public Trigger exhaustTrigger();
    public Trigger windUpTrigger();
    public Trigger unwindTrigger();

    public Trigger climbRotateTenTimes();
    public Trigger climbWithJoySticks();
}
