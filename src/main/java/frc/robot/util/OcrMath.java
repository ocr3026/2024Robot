package frc.robot.util;

public class OcrMath {
        public static double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    
}
