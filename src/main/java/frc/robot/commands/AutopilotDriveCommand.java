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

/**
 * Command to drive the robot in autopilot mode, automatically adjusting based on inferred
 * robot position and driver inputs.
 */
public class AutopilotDriveCommand extends A05DriveCommand {

	/**
	 * Current driver intention, used to assess agreement with the autopilot.
	 */
	protected double intention;

	/**
	 * Represents the robot's inferred position on the field.
	 */
	protected InferredRobotPosition inferredRobotPosition = InferredRobotPosition.INVALID_IRP;

	/**
	 * The set of April tags being targeted by the robot for positioning.
	 */
	protected A05Constants.AprilTagSet tagSet = Constants.aprilTagSetDictionary.get("source close");

	/**
	 * Threshold for making driving corrections based on driver input.
	 */
	protected final double makeCorrectionsThreshold = 0.5;

	/**
	 * Tracks whether corrections were made based on driver input.
	 */
	protected boolean madeCorrections;

	/**
	 * Threshold for angular difference between target and joystick direction.
	 */
	protected final double DIRECTION_DELTA_THRESHOLD = 35.0;

	/**
	 * Weight applied to driver agreement factor in calculations.
	 */
	protected final double DRIVER_AGREEMENT_WEIGHT = makeCorrectionsThreshold / 5.0;

	/**
	 * Timeout threshold for handling driver disagreement situations.
	 */
	protected final int DRIVER_DISAGREEMENT_TIMEOUT_THRESHOLD = 150;

	/**
	 * Accumulated excess agreement score when the driver consistently aligns with autopilot.
	 */
	protected int excessAgreement;

	/**
	 * Timeout counter for periods of driver disagreement.
	 */
	protected int driverDisagreementTimeout;

	/**
	 * Constructs an AutopilotDriveCommand.
	 *
	 * @param swerveDrive The swerve drive subsystem used by this command.
	 */
	public AutopilotDriveCommand(ISwerveDrive swerveDrive) {
		super(swerveDrive);
		addRequirements();
	}

	/**
	 * Initializes the autopilot command, resetting internal states.
	 */
	@Override
	public void initialize() {
		super.initialize();
		excessAgreement = 0;
		driverDisagreementTimeout = 0;
		madeCorrections = false;
		intention = 0;
	}

	/**
	 * Executes the command, computing driver agreement and making adjustments based on inferred position.
	 */
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

		// Calculate the direction towards the targeted April Tag position
		AngleD targetingDirection = new AngleD().atan2(
				inferredRobotPosition.y - tagSet.DEFAULT_Y_POSITION,
				inferredRobotPosition.x - tagSet.DEFAULT_X_POSITION
		);

		// Calculate the direction based on joystick input with dead-zone handling
		double altX = Utl.inTolerance(Constants.DRIVE_XBOX.getLeftX(), 0.0, 0.05) ? 0.0 : Constants.DRIVE_XBOX.getLeftX();
		double altY = Utl.inTolerance(Constants.DRIVE_XBOX.getLeftY(), 0.0, 0.05) ? 0.0 : -Constants.DRIVE_XBOX.getLeftY();
		AngleD joystickDirection = new AngleD().atan2(altX, altY);

		// Calculate driver agreement factor based on angular difference
		double driverAgreement = 1 - Math.abs(joystickDirection.cloneAngleD()
				.subtract(targetingDirection)
				.getDegrees() / DIRECTION_DELTA_THRESHOLD);

		// Update intention based on driver agreement
		intention += (altY == 0.0 && altX == 0.0) ? 0.0 : driverAgreement * DRIVER_AGREEMENT_WEIGHT;

		// Adjust excess agreement and intention
		if(intention > 1.0) {
			excessAgreement = (int) Utl.clip(excessAgreement + 1, 0, 25);
		} else {
			excessAgreement = (int) Utl.clip(excessAgreement - 1, 0, 25);
			intention = excessAgreement > 0 ? 1.0 : intention;
		}

		// Handle corrections threshold and disagreement timeout
		if(intention > makeCorrectionsThreshold) {
			madeCorrections = true;
		} else if(madeCorrections && intention < makeCorrectionsThreshold) {
			driverDisagreementTimeout = DRIVER_DISAGREEMENT_TIMEOUT_THRESHOLD;
			madeCorrections = false;
		}

		// Limit intention to a 0.0 - 1.0 range
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

		// Adjust conditionedDirection based on intention and targeting difference
		if(intention > makeCorrectionsThreshold && !Constants.ALT_XBOX.getXButton()) {
			conditionedDirection.subtract(AngleUnit.DEGREES,
					joystickDirection.cloneAngleD()
							.subtract(targetingDirection)
							.getDegrees() *
							generalizedLogisticFunction((intention - makeCorrectionsThreshold) * (1 / makeCorrectionsThreshold), 2.3)
			);
		}

		iSwerveDrive.swerveDrive(conditionedDirection, conditionedSpeed, conditionedRotate);
	}

	/**
	 * Checks if the command has completed.
	 *
	 * @return true if the command is finished, false otherwise.
	 */
	@Override
	public boolean isFinished() {
		return super.isFinished();
	}

	/**
	 * Ends the command, stopping any autopilot adjustments.
	 *
	 * @param interrupted True if the command was interrupted, false if it finished normally.
	 */
	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
	}

	/**
	 * A generalized logistic function used for scaling corrections.
	 *
	 * @param value The input value to scale.
	 * @param a     The logistic function parameter.
	 * @return Scaled value between 0 and 1.
	 */
	protected double generalizedLogisticFunction(double value, double a) {
		return Math.pow(value, a) / (Math.pow(value, a) + Math.pow(1 - value, a));
	}
}
