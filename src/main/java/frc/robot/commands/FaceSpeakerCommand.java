package frc.robot.commands;

import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;


public class FaceSpeakerCommand extends A05DriveCommand {
    public FaceSpeakerCommand() {
        // NOTE: the super adds the drive subsystem requirement
        super(SpeedCachedSwerve.getInstance());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        conditionStick();
        navX.setExpectedHeading(navX.getHeadingInfo().getClosestUpField());
        conditionedRotate = new AngleD(navX.getHeadingInfo().expectedHeading).subtract(new AngleD(navX.getHeadingInfo().heading))
                .getRadians() * A05Constants.getDriveOrientationkp();
        conditionedRotate = Utl.clip(conditionedRotate, -0.5, 0.5);
        iSwerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
    }

    @Override
    public boolean isFinished() {
        return Utl.inTolerance(driveXbox.getRightX(), 0.0, 0.05); // Finish if the user inputs a rotation
    }

    @Override
    public void end(boolean interrupted) {
        navX.setExpectedHeadingToCurrent();
    }
}
