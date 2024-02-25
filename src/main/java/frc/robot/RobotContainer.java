// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.A05RobotContainer;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends A05RobotContainer {
    // The robot's subsystems and commands are defined here...
    // NavX, DriveSubsystem, DriveXbox have already been made in A05RobotContainer
    SpeedCachedSwerve speedCachedSwerve = SpeedCachedSwerve.getInstance();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        super();
        // finish swerve drive initialization for this specific robt.
        driveCommand = new DriveCommand(driveSubsystem);

        speedCachedSwerve.setDriveSubsystem(driveSubsystem);
        speedCachedSwerve.setCacheLength(1000);

        speedCachedSwerve.setDriveGeometry(robotSettings.length, robotSettings.width,
                robotSettings.rf, robotSettings.rr,
                robotSettings.lf, robotSettings.lr,
                robotSettings.maxSpeedCalibration);

        driveSubsystem.setDefaultCommand(driveCommand);
        ClimberSubsystem.getInstance().setDefaultCommand(new ManualClimberCommand());

        ArmSubsystem.getInstance().setDefaultCommand(new ManualArmCommand());

        // Configure the button bindings
        configureButtonBindings();
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * Joystick} or {@link XboxController}), and then passing it to a {@link
     * JoystickButton}.
     */
    private void configureButtonBindings() {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html

        driveBack.onTrue(new InstantCommand(navx::initializeHeadingAndNav)); // Reset the NavX field relativity

        altBack.onTrue(new InstantCommand(ArmSubsystem.getInstance()::toggleManualControl));

        driveY.whileTrue(new InstantCommand(ArmSubsystem.ArmPosition.PROTECTED::goTo));
        altY.whileTrue(new InstantCommand(ArmSubsystem.ArmPosition.PROTECTED::goTo));

        // All heading commands finish if the driver moves the rotate stick
        driveB.onTrue(new DynamicFaceRightCommand()); // Adjusts for color, faces amp or source, whichever is to the right
        driveA.onTrue(new FaceSpeakerCommand()); // Faces up-field, at speaker
        driveX.onTrue(new DynamicFaceLeftCommand()); // Adjusts for color, faces amp or source, whichever is to the left

        //driveRightBumper.onTrue(new GroundPickupCommand()).onFalse(new InstantCommand(CollectorSubsystem.getInstance()::stop));
        driveRightBumper.onTrue(new InstantCommand(ShooterSubsystem.getInstance()::shoot)).onFalse(new InstantCommand(ShooterSubsystem.getInstance()::stop));
        altRightBumper.whileTrue(new GroundPickupCommand()).onFalse(new InstantCommand(CollectorSubsystem.getInstance()::stop));

        altB.whileTrue(new DynamicTargetRightCommandGroup()); // Adjusts for color, targets amp or source, whichever is to the right
        altA.whileTrue(new SpeakerShootCommand()); // Scores at the speaker
        altX.whileTrue(new DynamicTargetLeftCommandGroup()); // Adjusts for color, targets amp or source, whichever is to the left

        //altLeftBumper.whileTrue(new ClimberRetractCommand());
        //driveLeftBumper.whileTrue(new ClimberRetractCommand());
        driveLeftBumper.onTrue(new InstantCommand(CollectorSubsystem.getInstance()::intake)).onFalse(new InstantCommand(CollectorSubsystem.getInstance()::stop));
    }
}
