package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.ShotLogger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;


public class SpeakerShootCommand extends DriveCommand {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    private final PhotonCameraWrapper camera = Constants.CAMERA;

    private final ShooterFeedCommand shooterFeedCommand = new ShooterFeedCommand();
    //TODO: test this value, may need to change
    private final double TARGET_ROTATION_KP = 0.8;
    private final A05Constants.AprilTagSet tagSet = Constants.aprilTagSetDictionary.get("speaker center");
    private boolean feedStarted = false;
    private int isFinishedDelay = 0;

    public SpeakerShootCommand() {
        super(SpeedCachedSwerve.getInstance());
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armSubsystem, this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        feedStarted = false;
        isFinishedDelay = 0;
        System.out.println("***************** STARTED *******************");
    }

    @Override
    public void execute() {
        // do we have new target data of the speaker?

        if(!camera.isTargetDataNew(tagSet)) {
            super.execute(); // Run normal drive algorithm
            return; // Wait till next tick
        }

        // we have a target, lets do the control math and edit the rotation to heading lock it.

        conditionStick();

        this.conditionedRotate = (camera.getTarget(tagSet).getYaw() - 4.6) / 35.0 * TARGET_ROTATION_KP;

        // lets linear interpolate to find arm and rpm numbers

        Constants.LinearInterpolation linearInterpolation = Constants.LinearInterpolation.interpolate(camera.getXFromLastTarget(tagSet));

        /*
          The LinearInterpolation class contains a value goodData. LinearInterpolation.interpolate was passed a distance
          and if that distance is outside the min or max interpolation points, goodData will be false.

          is the data inside an acceptable range or is the override button is pressed
        //*/

        if(!(linearInterpolation.goodData && Constants.ALT_XBOX.getPOV() == -1)) {
            iSwerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
            return;
        }

        // All conditions checked, stop robot movement and start moving arm and spinning shooter

        conditionedSpeed = 0.0;

        linearInterpolation.goToArm();
        ShooterSubsystem.getInstance().speaker();

        ShotLogger.storeShotData(linearInterpolation);

        iSwerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);

        // Is this the first tick that the arm and shooter are at the correct values?
        if(armSubsystem.isInPosition() && shooterSubsystem.getVelocity() > 5350.0) {
            // Feed in the note and mark feedStarted as true so we don't create infinite feed commands
            if(!feedStarted) {
                shooterFeedCommand.schedule();
            }
            feedStarted = true;
            isFinishedDelay++;
        }
    }

    @Override
    public boolean isFinished() {
        return feedStarted && !shooterFeedCommand.isScheduled() && isFinishedDelay >= 25;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
        CollectorSubsystem.getInstance().stop();
        ArmSubsystem.ArmPosition.PROTECTED.goTo();
    }
}
