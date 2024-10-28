package frc.robot.commands;

import org.a05annex.frc.InferredRobotPosition;
import org.a05annex.frc.commands.A05AprilTagPositionCommand;
import org.a05annex.frc.subsystems.PhotonCameraWrapper;
import org.a05annex.util.AngleD;
import org.a05annex.util.Utl;


public class TranslateAprilTagPositionCommand extends A05AprilTagPositionCommand {

    public TranslateAprilTagPositionCommand(PhotonCameraWrapper camera, double xPosition, double yPosition, String positionParametersKey) {
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

        inferredRobotPosition = InferredRobotPosition.getInferredRobotPosition(tagSet);

        checkIfCanPerformTargeting();


        if (this.canPerformTargeting) {
            this.canPerformTargeting = false;
//            this.conditionedSpeed = this.calcSpeed();
//            this.conditionedRotate = this.calcRotation();
//            this.calcDirection(this.conditionedDirection);
//            this.lastConditionedDirection = this.conditionedDirection;
//            this.lastConditionedSpeed = Utl.clip(this.conditionedSpeed, -0.25, 0.25);
//            this.lastConditionedRotate = this.conditionedRotate;
            swerveDrive.translate(inferredRobotPosition.x - X_POSITION, inferredRobotPosition.y - Y_POSITION);
            if (this.checkInZone()) {
                ++this.ticksInZoneCounter;
                //this.swerveDrive.swerveDrive(AngleD.ZERO, 0.0, this.conditionedRotate * 0.1);
                if (this.ticksInZoneCounter > 10) {
                    this.isFinished = true;
                }

            } else {
                //this.swerveDrive.swerveDrive(this.conditionedDirection, this.conditionedSpeed, this.conditionedRotate);
            }
        }
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
    protected double calcY() {
        double scale = (this.Y_MAX - this.Y_MIN) / 2.0;
        return -Utl.clip((this.inferredRobotPosition.y - this.Y_POSITION) / scale, -1.0, 1.0);
    }
}
