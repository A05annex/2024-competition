// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ClimberTensionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.A05Robot;
import org.a05annex.frc.NavX;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Collections;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends A05Robot {
    public void permanentTelemetry() {
        SmartDashboard.putNumber("heading", NavX.getInstance().getHeading().getDegrees());
        SmartDashboard.putBoolean("red?", NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true));
        SmartDashboard.putNumber("analog encoder", Constants.ARM_ANALOG_ENCODER.getAbsolutePosition());
        SmartDashboard.putBoolean("Note Sensor", Constants.NOTE_SENSOR.get());

        SmartDashboard.putNumber("collector rpm", CollectorSubsystem.getInstance().getRpm());
        SmartDashboard.putNumber("Shooter rpm", ShooterSubsystem.getInstance().getVelocity());
        SmartDashboard.putNumber("rihtShooter", ShooterSubsystem.getInstance().getRightVelocity());
        SmartDashboard.putNumber("forward arm encoder", ArmSubsystem.getInstance().getFrontPos());
        SmartDashboard.putNumber("backward arm encoder", ArmSubsystem.getInstance().getBackPos());

        SmartDashboard.putData(CommandScheduler.getInstance());

        SmartDashboard.putBoolean("Manual Arm", ArmSubsystem.getInstance().manualControl());

        SmartDashboard.putNumber("left climber", ClimberSubsystem.getInstance().getLeftPosition());
        SmartDashboard.putNumber("left rpm", ClimberSubsystem.getInstance().getLeftRpm());
        SmartDashboard.putNumber("right climber", ClimberSubsystem.getInstance().getRightPosition());
        SmartDashboard.putNumber("right rpm", ClimberSubsystem.getInstance().getRightRpm());

        if(Constants.CAMERA.camera.isConnected()) {
            Constants.CAMERA.updateTrackingData();
            //SmartDashboard.putNumber("Distance", Constants.CAMERA.getXFromLastTarget(Constants.aprilTagSetDictionary.get("speaker center")));
            SmartDashboard.putBoolean("newest frame targs", Constants.CAMERA.getNewestFrame().hasTargets());
            if(Constants.CAMERA.camera.hasTargets()) {
                SmartDashboard.putNumber("Distance", Constants.CAMERA.getXFromLastTarget(Constants.aprilTagSetDictionary.get("amp")) * Constants.CAMERA_ANGLE.cos());
            }
        } else {
            //SmartDashboard.putNumber("Distance", -1.0);
            SmartDashboard.putBoolean("newest frame targs", false);
        }

        SpeedCachedSwerve.getInstance().setLatencyOffset(SmartDashboard.getNumber("latency", SpeedCachedSwerve.getInstance().getLatencyOffset()));
        SmartDashboard.putNumber("real latency", SpeedCachedSwerve.getInstance().getLatencyOffset());
    }

    public void enabledTelemetry() {
        permanentTelemetry();
    }

    public void disabledTelemetry() {
        permanentTelemetry();
    }

    public void enableInit() {
        ArmSubsystem.getInstance().enableInit();
        new ClimberTensionCommand().schedule();
    }

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        super.robotInit();

        Constants.setSparkConfig(true, false);

        // Set the drive constants that are specific to this swerve geometry.
        // Some drive geometry is passed in RobotContainer's constructor
        Constants.setDriveOrientationkp(Constants.DRIVE_ORIENTATION_kP);

        Constants.setPrintDebug(false);

        // update dictionary with all needed values
        Constants.setAprilTagSetDictionary();

        SpeedCachedSwerve.getInstance().setLogging(true);

        // Load the robot settings list
        Collections.addAll(A05Constants.ROBOT_SETTINGS_LIST, Constants.ROBOT_SETTINGS);
        // Load the autonomous path list
        Collections.addAll(A05Constants.AUTONOMOUS_PATH_LIST, Constants.AUTONOMOUS_PATHS);
        // Load the driver list
        Collections.addAll(A05Constants.DRIVER_SETTINGS_LIST, Constants.DRIVER_SETTINGS);

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        setRobotContainer(new RobotContainer());
    }


    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        ArmSubsystem.getInstance().stop();
    }


    @Override
    public void disabledPeriodic() {
        disabledTelemetry();
    }

    PhotonTrackedTarget initTarg;
    double time;

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        enableInit();

        initTarg = Constants.CAMERA.getTarget(Constants.aprilTagSetDictionary.get("amp"));
        time = Constants.CAMERA.getLatestTargetTime();
        // Sets up autonomous command
        super.autonomousInit();
    }

    public static final String APRIL_TAG_LOG_NAME = "aprilTag";
    private final StringLogEntry aprilTagLog = new StringLogEntry(DataLogManager.getLog(),APRIL_TAG_LOG_NAME);

    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        enabledTelemetry();

        Constants.CAMERA.updateTrackingData();
        if(Constants.CAMERA.camera.hasTargets()) {
            SpeedCachedSwerve.RobotRelativePosition position = null;
            try {
                position = SpeedCachedSwerve.getInstance().getRobotRelativePositionSince(time);
            } catch (Exception e) {
            }

            aprilTagLog.append(String.format("TRUE,%.4f,%.5f,%.5f,%.5f,%.5f",Constants.CAMERA.getLatestTargetTime(),
                    Constants.CAMERA.getXFromLastTarget(Constants.aprilTagSetDictionary.get("amp")), Constants.CAMERA.getYFromLastTarget(Constants.aprilTagSetDictionary.get("amp")),
                    initTarg.getBestCameraToTarget().getX() - (position == null ? 0.0 : position.forward), initTarg.getBestCameraToTarget().getY() - (position == null ? 0.0 : position.strafe)));
                    //0.0, 0.0));
        }
    }


    @Override
    public void teleopInit() {
        if(!a05RobotContainer.driveRightStickPress.getAsBoolean()) {
            enableInit();
        }
        // Cancels autonomous command
        super.teleopInit();
    }


    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        super.teleopPeriodic();

        enabledTelemetry();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }


    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        enabledTelemetry();
    }
}
