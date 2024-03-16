package frc.robot.util;

import java.util.Calendar;

import frc.robot.Constants;

public final class TunaLogger {
    public static void log(String str) {
        if(!Constants.tunaFish) {
            return;
        }
        
        Calendar cal = Calendar.getInstance();

        System.out.format("[%d:%d:%d] %s", cal.get(Calendar.HOUR_OF_DAY), cal.get(Calendar.MINUTE), cal.get(Calendar.SECOND),  str);
    }
}
