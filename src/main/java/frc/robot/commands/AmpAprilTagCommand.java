package frc.robot.commands;

import org.a05annex.frc.InferredRobotPosition;
import org.a05annex.frc.commands.A05AprilTagPositionCommand;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;


public class AmpAprilTagCommand extends A05AprilTagPositionCommand {

    public AmpAprilTagCommand(double xPosition, double yPosition, String positionParametersKey) {
        // NOTE: the super adds the drive subsystem requirement
        super(xPosition, yPosition, positionParametersKey);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        // Similar to the drive command, you can either call the super.execute which runs checkIfCanPerformTargeting()
        // and executeTargeting(), or write your own code.

        // super.execute();

        // NOTE: there is a variable called 'canPerformTargeting' that needs to get set to true in order to use
        // executeTargeting(). checkIfCanPerformTargeting will set this to true if the conditions in it are met

        checkIfCanPerformTargeting();


        if(!canPerformTargeting) {
            return;
        }
        inferredRobotPosition = InferredRobotPosition.getInferredRobotPosition(tagSet);


        executeTargeting();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    protected double calcRotationFieldHeading() {
        AngleD fieldHeading = navX.getHeadingInfo().getClosestHeading(HEADING);
        navX.setExpectedHeading(fieldHeading);
        return new AngleD(navX.getHeadingInfo().expectedHeading).
                subtract(new AngleD(navX.getHeadingInfo().heading)).getRadians() * HEADING_ROTATION_KP;
    }
}
