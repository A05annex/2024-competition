package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.SpeedCachedSwerve;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;


public class DynamicFaceLeftCommand extends A05DriveCommand {

    public DynamicFaceLeftCommand() {
        // NOTE: the super adds the drive subsystem requirement
        super(SpeedCachedSwerve.getInstance());
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        conditionStick();

        double heading = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true) ? 300 : 270;
                                                                                                                       // 90 = red amp, 300 = blue source
        navX.setExpectedHeading(navX.getHeadingInfo().getClosestHeading(new AngleD(AngleUnit.DEGREES, heading)));
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
