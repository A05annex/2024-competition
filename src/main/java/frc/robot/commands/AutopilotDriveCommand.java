package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.a05annex.frc.A05Constants;
import org.a05annex.frc.InferredRobotPosition;
import org.a05annex.frc.commands.A05DriveCommand;
import org.a05annex.frc.subsystems.ISwerveDrive;
import org.a05annex.util.AngleD;
import org.a05annex.util.AngleUnit;
import org.a05annex.util.Utl;


public class AutopilotDriveCommand extends A05DriveCommand {
    protected double intention;

    protected InferredRobotPosition inferredRobotPosition = InferredRobotPosition.INVALID_IRP;


    protected A05Constants.AprilTagSet tagSet = Constants.aprilTagSetDictionary.get("source close");


    protected double makeCorrectionsThreshold = 0.5;
    protected boolean madeCorrections;

    protected final double DIRECTION_DELTA_THRESHOLD = 35.0;

    protected final double DRIVER_AGREEMENT_WEIGHT = makeCorrectionsThreshold / 5.0;
    protected final int DRIVER_DISAGREEMENT_TIMEOUT_THRESHOLD = 50;

    protected int excessAgreement;
    protected int driverDisagreementTimeout;

    public AutopilotDriveCommand(ISwerveDrive swerveDrive) {
        super(swerveDrive);
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements();
    }

    @Override
    public void initialize() {
        super.initialize();
        excessAgreement = 0;
        driverDisagreementTimeout = 0;
        madeCorrections = false;
    }

    @Override
    public void execute() {
        inferredRobotPosition = InferredRobotPosition.getRobotPosition(tagSet);

        if(inferredRobotPosition == InferredRobotPosition.INVALID_IRP) {
            super.execute();
            return;
        }

        if(driverDisagreementTimeout > 0) {
            driverDisagreementTimeout--;
            SmartDashboard.putNumber("driver disagreement timeout", driverDisagreementTimeout);
            super.execute();
            return;
        }

        conditionStick();

        AngleD targetingDirection = new AngleD().atan2(inferredRobotPosition.y - tagSet.DEFAULT_Y_POSITION,
                inferredRobotPosition.x - tagSet.DEFAULT_X_POSITION);

        double altX = Utl.inTolerance(Constants.DRIVE_XBOX.getLeftX(), 0.0, 0.05) ? 0.0 : Constants.DRIVE_XBOX.getLeftX();
        double altY = Utl.inTolerance(Constants.DRIVE_XBOX.getLeftY(), 0.0, 0.05) ? 0.0 : -Constants.DRIVE_XBOX.getLeftY();

        AngleD joystickDirection = new AngleD().atan2(altX, altY);

        double driverAgreement = 1 - Math.abs(joystickDirection.cloneAngleD().subtract(targetingDirection).getDegrees() / DIRECTION_DELTA_THRESHOLD);

        intention += (altY == 0.0) && (altX == 0) ? 0.0 : driverAgreement * DRIVER_AGREEMENT_WEIGHT;

        if(intention > 1.0) {
            excessAgreement = (int) Utl.clip(excessAgreement + 1, 0, 25);
        } else {
            excessAgreement = (int) Utl.clip(excessAgreement - 1, 0, 25);

            intention = excessAgreement > 0 ? 1.0 : intention;
        }

        if(intention > makeCorrectionsThreshold) {
            madeCorrections = true;
        } else if(madeCorrections && intention < makeCorrectionsThreshold) {
            driverDisagreementTimeout = DRIVER_DISAGREEMENT_TIMEOUT_THRESHOLD;
            madeCorrections = false;
        }




        intention = Utl.clip(intention, 0.0, 1.0);

        SmartDashboard.putNumber("ALT Left X", Constants.ALT_XBOX.getLeftX());
        SmartDashboard.putNumber("ALT Left Y", Constants.ALT_XBOX.getLeftY());

        SmartDashboard.putNumber("joystick Direction", joystickDirection.getDegrees());
        SmartDashboard.putNumber("target Direction", targetingDirection.getDegrees());
        SmartDashboard.putNumber("Driver Agreement", driverAgreement);
        SmartDashboard.putNumber("direction difference", joystickDirection.cloneAngleD().subtract(targetingDirection).getDegrees());
        SmartDashboard.putNumber("intention", intention);
        SmartDashboard.putNumber("excess agreement", excessAgreement);
        SmartDashboard.putBoolean("make corrections threshold", madeCorrections);
        SmartDashboard.putNumber("driver disagreement timeout", driverDisagreementTimeout);

        if(intention > makeCorrectionsThreshold && !Constants.ALT_XBOX.getXButton()) {
            conditionedDirection.subtract(AngleUnit.DEGREES, joystickDirection.cloneAngleD(). //Make a copy so we don't mess up the angle
                    subtract(targetingDirection).getDegrees() * // Get the difference between the joystick and the target
                    generalizedLogisticFunction((intention - makeCorrectionsThreshold) * (1/makeCorrectionsThreshold), 2.3));
                    // Use a generalized logistic function to scale the difference. Intention is scaled to the range of 0 to 1 as the input
        }

        //conditionedSpeed = Utl.clip(conditionedSpeed, -0.05, 0.05);

        iSwerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    protected double generalizedLogisticFunction(double value, double a) {
        return Math.pow(value, a) / (Math.pow(value, a) + Math.pow(1 - value, a));
    }
}
