package frc.robot.util;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort;


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
		return (float)MathUtil.inputModulus(super.getYaw() + gyroOffset, -180, 180);
	}

	@Override
	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(getYaw());
	}

	public void setYaw(float angle) {
		super.zeroYaw();
		gyroOffset = angle;
	}
}