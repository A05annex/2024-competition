// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ArmSubsystem;
import org.a05annex.frc.A05Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.photonvision.PhotonCamera;

import javax.sound.sampled.Line;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants extends A05Constants
{
    public static final class CAN_Devices {
        public static final int
            // Non-Drive Motors
                FORWARD_ARM_MOTOR = 14,
                BACKWARD_ARM_MOTOR = 13;
    }


    public static final class LinearInterpolation {
        public final double distance;
        public final double arm;
        public final double rpm;

        private static final LinearInterpolation[] calibratedPoints = {
                new LinearInterpolation(1.0, 100, 5000),
                new LinearInterpolation(2.0, 150, 5100)
        };

        LinearInterpolation(double distance, double arm, double rpm) {
            this.distance = distance;
            this.arm = arm;
            this.rpm = rpm;
        }

        public void goToArm() {
            ArmSubsystem.getInstance().goToSmartMotionPosition(this.arm);
        }

        public static LinearInterpolation interpolate(double distance) {
            int highIndex = 0; // Index of the first calibrated point the distance parameter is less than


            // Find which two calibrated points the current robot distance is between
            for(LinearInterpolation calibratedPoint : calibratedPoints) {
                if(distance < calibratedPoint.distance) {
                    break;
                }
                highIndex++;
            }

            // The distance parameter was less than the first point, so just return the first point
            if(highIndex == 0) {
                return calibratedPoints[highIndex];
            }

            // The distance parameter was greater than the last point, so just return the last point
            if(highIndex == calibratedPoints.length) {
                return calibratedPoints[calibratedPoints.length - 1];
            }

            int lowIndex = highIndex - 1; // Index of the last calibrated point the distance parameter is greater than

            double arm = calibratedPoints[lowIndex].arm // Value to find change from
                    + ((distance - calibratedPoints[lowIndex].distance) / (calibratedPoints[highIndex].distance - calibratedPoints[lowIndex].distance)) // Percent change formula
                    * (calibratedPoints[highIndex].arm - calibratedPoints[lowIndex].arm); // Change in arm

            double rpm = calibratedPoints[lowIndex].rpm // Value to find change from
                    + ((distance - calibratedPoints[lowIndex].distance) / (calibratedPoints[highIndex].distance - calibratedPoints[lowIndex].distance)) // Percent change formula
                    * (calibratedPoints[highIndex].rpm - calibratedPoints[lowIndex].rpm); // Change in rpm

            return new LinearInterpolation(distance, arm, rpm);
        }
    }

    public static final boolean HAS_USB_CAMERA = false;
    public static final boolean HAS_LIMELIGHT = false;


    public static final PhotonCameraWrapper CAMERA = new PhotonCameraWrapper(new PhotonCamera("IMX219"), 0.127, new AngleD(AngleUnit.DEGREES, 45));

    // kP for keeping drive at the same orientation
    public static double DRIVE_ORIENTATION_kP = 1.2;

    // for practice, length and width from center of the wheels, in m (note chassis is 30" square,
    // the bolt pattern is 29" square, wheels are 2.75" in from the bolt pattern or centered on the
    // corners of a 23.5"(0.5969m) square.
    // For competition, length and width from center of the wheels, in m (note chassis is 28" square,
    // the bolt pattern is 27" square, wheels are 2.75" in from the bolt pattern or centered on the
    // corners of a 21.5"(0.5461m) square.
    /**
     * The geometry and calibration specific to a swerve drive robot base. We currently have 2 bases, the first being
     * a <i>prototyping/practice</i> base that should always be in working condition for drive tuning/testing,
     * calibration, as well as software prototyping. The second is the <i>competition</i> robot that is built for
     * the yearly competition, and is generally not drivable between the introduction of the competition and a
     * week or two before the first competition because all the competition-specific appendages are being built
     * and assembled to it.
     *
     * The <i>competition</i> robot is the index 0 entry in the list, so it does not require any changes to the
     * Roborio. The <i>practice</i> robot requires a jumper on the digital input port 5 that connects the signal
     * pin to ground. This convention was chosen to minimize the things that could go wrong on the competition robot.
     *
     * These settings are loaded into {@link #ROBOT_SETTINGS_LIST} during {@link Robot#robotInit()}
     */
    public static final A05Constants.RobotSettings[] ROBOT_SETTINGS = {
            new A05Constants.RobotSettings(0, "Competition", 0.5461, 0.5461, 2.700, 1.161,
                    2.723, 2.448, 1.026,0.9650),
            new A05Constants.RobotSettings(1, "Practice", 0.5969, 0.5969, 5.240, 5.654,
                    0.969, 5.039, 1.026, 0.9164)
    };

    public static final A05Constants.AutonomousPath[] AUTONOMOUS_PATHS = {
            new A05Constants.AutonomousPath("Sample Path", 0, "samplePath.json")
    };

    public static final A05Constants.DriverSettings[] DRIVER_SETTINGS = {
            new DriverSettings("programmer", 0)
    };

    public static void setAprilTagSetDictionary() {
        aprilTagSetDictionary.put("amp", new AprilTagSet(new int[] {5}, new int[] {6},1.355725, new AngleD(AngleUnit.DEGREES, 90.0), new AngleD(AngleUnit.DEGREES, 270.0)));
        aprilTagSetDictionary.put("speaker center", new AprilTagSet(new int[] {4}, new int[] {7}, 1.450975));
        aprilTagSetDictionary.put("speaker offset", new AprilTagSet(new int[] {3},  new int[] {8}, 1.450975));
        aprilTagSetDictionary.put("source close", new AprilTagSet(new int[] {10}, new int[] {1}, 1.355725, new AngleD(AngleUnit.DEGREES, 300.0), new AngleD(AngleUnit.DEGREES, 60.0)));
        aprilTagSetDictionary.put("source far", new AprilTagSet(new int[] {9}, new int[] {2}, 1.355725, new AngleD(AngleUnit.DEGREES, 300.0), new AngleD(AngleUnit.DEGREES, 60.0)));
        aprilTagSetDictionary.put("stage far", new AprilTagSet(new int[] {13}, new int[] {14}, 1.3208, new AngleD(AngleUnit.DEGREES, 180.0)));
        aprilTagSetDictionary.put("stage left", new AprilTagSet(new int[] {11}, new int[] {15}, 1.3208, new AngleD(AngleUnit.DEGREES, 60.0)));
        aprilTagSetDictionary.put("stage right", new AprilTagSet(new int[] {12}, new int[] {16}, 1.3208, new AngleD(AngleUnit.DEGREES, 300.0)));
    }


    // Connect values to SmartDashboard, if you change the value in smart dashboard it changes the constant
    // (speed adjusting etc.) By having two methods, you can optionally add the bounds
    /**
     * Initialize value on SmartDashboard for user input, or if already present, return current value.
     *
     * @param key (String) The key to associate with the value.
     * @param initValue (double) The default value to assign if not already on SmartDashboard.
     *
     * @return The new value that appears on the dashboard.
     */
    @SuppressWarnings("unused")
    public static double updateConstant(String key, double initValue) {
        // if key already exists, value will be the current value or whatever we just typed in to the dashboard
        // if key doesn't exist yet, value will be set to initValue and added to SmartDashboard
        double value = SmartDashboard.getNumber(key, initValue);

        // add number if it doesn't exist, or just set it to its current value
        SmartDashboard.putNumber(key, value);
        return value;
    }

    /**
     * Initialize value on SmartDashboard for user input, or if already present, return current value.
     * If value is outside (lowerBound, upperBound), it will be set to the previous value.
     *
     * @param key (String) The key to associate with the value.
     * @param initValue (double) The default value to assign if not already on SmartDashboard.
     * @param lowerBound (double) Lower bound on the value.
     * @param upperBound (double) Upper bound on the value.
     *
     * @return The new value that appears on the dashboard.
     */
    @SuppressWarnings("unused")
    public static double updateConstant(String key, double initValue, double lowerBound, double upperBound) {
        // if key already exists, value will be the current value or whatever we just typed in to the dashboard
        // if key doesn't exist yet, value will be set to initValue and added to SmartDashboard
        double value = SmartDashboard.getNumber(key, initValue);

        // bounds check
        if (value < lowerBound || value > upperBound) {
            value = initValue;
        }

        // add number if it doesn't exist, or just set it to its current value
        SmartDashboard.putNumber(key, value);
        return value;
    }
}
