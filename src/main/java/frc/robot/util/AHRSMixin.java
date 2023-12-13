package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AHRSMixin extends AHRS {
	float gyroOffset = 0;

	public AHRSMixin(SerialPort.Port port) { super(port); }

	@Override
	public void zeroYaw() {
		super.zeroYaw();
		gyroOffset = 0;
	}

	@Override
	public float getYaw() {
		float angle = (float)MathUtil.inputModulus(super.getYaw() + gyroOffset, -180, 180);
		SmartDashboard.putNumber("gyroAngle", angle);
		return angle;
	}

	public void setYaw(float angle) {
		super.zeroYaw();
		gyroOffset = angle;
	}
}