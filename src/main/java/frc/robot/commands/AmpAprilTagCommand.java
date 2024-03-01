package frc.robot.commands;

import org.a05annex.frc.commands.A05AprilTagPositionCommand;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.util.Utl;


public class AmpAprilTagCommand extends A05AprilTagPositionCommand {

    public AmpAprilTagCommand(PhotonCameraWrapper camera, double xPosition, double yPosition, String positionParametersKey) {
        // NOTE: the super adds the drive subsystem requirement
        super(camera, xPosition, yPosition, positionParametersKey);
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

        camera.updateTrackingData();
        checkIfCanPerformTargeting();


        positionAtFrame = swerveDrive.getRobotRelativePositionSince(camera.getLatestTargetTime());



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
    protected double calcSpeed() {
        double speed = Math.pow(Math.sqrt(Math.pow(Math.abs(calcX()), 2) + Math.pow(Math.abs(calcY()), 2)), SPEED_SMOOTHING_MULTIPLIER);

        // Limit the speed delta
        speed = Utl.clip(speed, lastConditionedSpeed - MAX_SPEED_DELTA, lastConditionedSpeed + MAX_SPEED_DELTA);

        // Slows the robot down as we go longer without a target. Hopefully allows the robot to "catch" the target again
        //speed *= ((double) (resumeDrivingTickThreshold - ticksWithoutTarget) / (double) resumeDrivingTickThreshold);
        // Don't slow the robot down because we want to keep moving to position

        // Clip robot speed to stay below max speed
        return Utl.clip(speed, 0.0, MAX_SPEED);
    }
}
