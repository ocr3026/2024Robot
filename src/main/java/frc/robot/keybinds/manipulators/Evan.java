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
        return Constants.xbox.x();
    }

    @Override 
    public Trigger exhaustTrigger() {
        return Constants.xbox.y();
    }
    @Override 
    public Trigger windUpTrigger() {
        return Constants.xbox.a();
    }
    @Override
    public Trigger unwindTrigger() {
        return Constants.xbox.b();
    }

    @Override
    public Trigger climbRotateTenTimes() {
        return Constants.xbox.back();
    }

    @Override
    public Trigger climbWithJoySticks() {
        return Constants.xbox.rightBumper();
    }

    @Override
    public Trigger ampTrigger () {
        return Constants.xbox.leftTrigger();
    }

    @Override
    public Trigger servoTrigger () {
        return Constants.xbox.leftBumper();
    }

}
