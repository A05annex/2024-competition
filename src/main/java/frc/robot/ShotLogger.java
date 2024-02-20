package frc.robot;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class ShotLogger {
    private static double distance = -1.0, rpm, arm;
    private static boolean lastShotLogged;

    private static final StringLogEntry shotLog = new StringLogEntry(DataLogManager.getLog(), "shotLog");

    private enum Outcome {
        SCORE,
        MISS,
        UNKNOWN;
    }

    public static void storeShotData(Constants.LinearInterpolation point) {
        if(!lastShotLogged) {
            // Nobody entered an outcome since the last shot was taken. Log it as unknown.
            performLogEntry(Outcome.UNKNOWN);
        }

        distance = point.distance;
        rpm = point.rpm;
        arm = point.arm;
        lastShotLogged = false;
    }

    public static void shotScored() {
        if(lastShotLogged) {
            // Oops! you tried to input an outcome before a shot was even taken
            return;
        }

        performLogEntry(Outcome.SCORE);
    }

    public static void shotMissed() {
        if(lastShotLogged) {
            // Oops! you tried to input an outcome before a shot was even taken
            return;
        }

        performLogEntry(Outcome.MISS);
    }

    private static void performLogEntry(Outcome outcome) {
        if(distance == -1.0) {
            shotLog.append("This is the beginning of the log. Below are the linear interpolation constants being used.");

            for(Constants.LinearInterpolation linInterp : Constants.LinearInterpolation.calibratedPoints) {
                shotLog.append(String.format("Distance: %f, Rpm: %f, Arm Position: %f", linInterp.distance, linInterp.rpm, linInterp.arm));
            }

            shotLog.append("***** Below are shots taken along with outcomes. UNKNOWN means that the next shot was taken before an outcome was input");
        }

        lastShotLogged = true;

        shotLog.append(String.format("Distance: %f, Rpm: %f, Arm Position: %f, Outcome: %s", distance, rpm, arm, outcome.toString()));
    }
}
