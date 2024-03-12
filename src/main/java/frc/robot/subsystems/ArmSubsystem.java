package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CLIMBER_ARM_STATUS;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.util.Utl;

public class ArmSubsystem extends SubsystemBase {

    private final static ArmSubsystem INSTANCE = new ArmSubsystem();
    // Position most recently requested of the arm
    private static double requestedPosition = 0.0;
    private final SparkNeo forwardMotor = SparkNeo.factory(Constants.CAN_Devices.FORWARD_ARM_MOTOR);
    private final SparkNeo backwardMotor = SparkNeo.factory(Constants.CAN_Devices.BACKWARD_ARM_MOTOR);
    // Declare PID constants for smart motion control
    private final double
            smKp = 0.0003,
    //smKp = 0.000025,
    smKi = 0.0001, smKiZone = 0.2, smKff = 0.000156, smMaxRPM = 3000.0,
            smMaxDeltaRPMSec = 3000.0, smMinRPM = 0.0, smError = 0.1, smKd = 0.0;
    // Declare PID constants for position control
    private final double posKp = 0.00005, posKi = 0.0, posKiZone = 0.0, posKff = 0.0;
    // Declare PID constants for speed (rpm) control
    private final double rpmKp = 0.5, rpmKi = 0.0, rpmKiZone = 0.0, rpmKff = 0.0;
    // Declare min and max soft limits and where the motor thinks it starts
    private final Double minPosition = -1.0, maxPosition = 34.0;
    // Tolerance to decide if in position
    private final double IN_POSITION_DEADBAND = 0.5;
    private final double ANALOG_ENCODER_ZERO = 0.9307;
    private final int gearRatio = 100;
    private boolean enableInit = false;
    private boolean manualControl = false;

    private ArmSubsystem() {
        forwardMotor.startConfig();
        forwardMotor.setCurrentLimit(SparkNeo.UseType.POSITION, SparkNeo.BreakerAmps.Amps40);
        forwardMotor.setSoftLimits(minPosition, maxPosition);
        forwardMotor.setDirection(SparkNeo.Direction.DEFAULT);
        forwardMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        forwardMotor.setPositionPID(posKp, posKi, posKiZone, posKff);
        forwardMotor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        forwardMotor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        forwardMotor.endConfig();
        forwardMotor.setEncoderPosition((Constants.getArmEncoder() - ANALOG_ENCODER_ZERO) * gearRatio);

        backwardMotor.startConfig();
        backwardMotor.setCurrentLimit(SparkNeo.UseType.POSITION, SparkNeo.BreakerAmps.Amps40);
        backwardMotor.setSoftLimits(minPosition, maxPosition);
        backwardMotor.setDirection(SparkNeo.Direction.REVERSE);
        backwardMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        backwardMotor.setPositionPID(posKp, posKi, posKiZone, posKff);
        backwardMotor.setSmartMotion(smKp, 0.0, 0.0, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError); // 0.0 kI because we don't want two motors with kI
        backwardMotor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        backwardMotor.endConfig();
        backwardMotor.setEncoderPosition((Constants.getArmEncoder() - ANALOG_ENCODER_ZERO) * gearRatio);

        //forwardMotor.setEncoderPosition((Constants.getArmEncoder() - armSubsystem.ANALOG_ENCODER_ZERO) * gearRatio);
        //backwardMotor.setEncoderPosition((Constants.getArmEncoder() - armSubsystem.ANALOG_ENCODER_ZERO) * gearRatio);
    }

    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    //public void double backwardMotor.SampleMotorSubsystem null (0.0);:

    public void enableInit() {
        if(enableInit) {
            return;
        }

        //ArmPosition.START.goTo();

        double startTime = Timer.getFPGATimestamp();
        System.out.println("****************************************************************");
        System.out.println("ENABLE INIT STARTED: " + startTime);
        System.out.println("****************************************************************");

        forwardMotor.setEncoderPosition(0.0);
        backwardMotor.setEncoderPosition(0.0);

        // Lock the forward (supporting) motor to start pos.
        forwardMotor.setTargetPosition(0.0);
        System.out.println("****************************************************************");
        System.out.printf("TIME: %f; support = %f; tension = %f%n",
                Timer.getFPGATimestamp() - startTime, forwardMotor.getEncoderPosition(), backwardMotor.getEncoderPosition());

        // 0.5 amps, just enough to tension
        backwardMotor.sparkMaxPID.setReference(1.2, CANSparkMax.ControlType.kVoltage);

        while(true) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                break;
            }

            double currentPos = forwardMotor.getEncoderPosition();

            System.out.printf("TIME: %f; support = %f; tension = %f;",
                    Timer.getFPGATimestamp() - startTime, currentPos, backwardMotor.getEncoderPosition());


            // Repeat until the voltage motor moves the position motor
            if(currentPos > 0.0) {
                break;
            }
        }

        //Removed play and resetting the backward encoder, so the tension can be kept
        forwardMotor.setEncoderPosition((Constants.getArmEncoder() - ANALOG_ENCODER_ZERO) * gearRatio);
        backwardMotor.setEncoderPosition((Constants.getArmEncoder() - ANALOG_ENCODER_ZERO) * gearRatio);

        System.out.printf("END TIME: %f; support = %f; tension = %f%n",
                Timer.getFPGATimestamp() - startTime, forwardMotor.getEncoderPosition(), backwardMotor.getEncoderPosition());
        System.out.println("****************************************************************");
        System.out.println("****************************************************************");
        System.out.println("****************************************************************");

        // Hold the arms at the start position
        forwardMotor.setSmartMotionTarget((Constants.getArmEncoder() - ANALOG_ENCODER_ZERO) * gearRatio);
        backwardMotor.setSmartMotionTarget((Constants.getArmEncoder() - ANALOG_ENCODER_ZERO) * gearRatio);

        requestedPosition = (Constants.getArmEncoder() - ANALOG_ENCODER_ZERO) * gearRatio;
        enableInit = true;
    }

    // Go to position with smart motion
    public void goToSmartMotionPosition(double position) {
        //Check to make sure the arm and climber are ok.
        if(Constants.getClimberArmStatus() == CLIMBER_ARM_STATUS.COLLISION) {
            /*
            It's very possible that the arm and climber are actively touching, and we don't know how, so its safest
            to just put the motors in brake mode.
            */
            stop();
            return;
        } else if(Constants.getClimberArmStatus() == CLIMBER_ARM_STATUS.DANGER) {
            /*
            The climber is higher than we want, and getting close to touching the arm. Move the arm to the protected
            position, just to be safe.
            */
            //forwardMotor.setSmartMotionTarget(ArmPosition.PROTECTED.position);
            //backwardMotor.setSmartMotionTarget(ArmPosition.PROTECTED.position);
            return;
        }

        forwardMotor.setSmartMotionTarget(position);
        backwardMotor.setSmartMotionTarget(position);
        requestedPosition = position;
    }

    // Go to position without smart motion
    public void goToPosition(double position) {
        forwardMotor.setTargetPosition(position);
        backwardMotor.setTargetPosition(position);
        requestedPosition = position;
    }

    public void goToDeltaPosition(double delta) {
        goToSmartMotionPosition(getPosition() + delta);
    }

    public void goToInterpolatedPosition(Constants.LinearInterpolation linearInterpolation) {
        goToSmartMotionPosition(linearInterpolation.arm);
    }

    // Stop motors (brake mode)
    public void stop() {
        forwardMotor.stopMotor();
        backwardMotor.stopMotor();
    }

    public double getPosition() {
        return forwardMotor.getEncoderPosition();
    }

    public double getFrontPos() {
        return forwardMotor.getEncoderPosition();
    }

    public double getBackPos() {
        return backwardMotor.getEncoderPosition();
    }

    // Returns if the arm is in the position the most recent requested position
    public boolean isInPosition() {
        return isInPosition(requestedPosition);
    }

    // Returns if the arm is in the position passed in
    public boolean isInPosition(double position) {
        return Utl.inTolerance(forwardMotor.getEncoderPosition(), position, IN_POSITION_DEADBAND);
    }

    public boolean manualControl() {
        return manualControl;
    }

    public void toggleManualControl() {
        manualControl = !manualControl;
    }

    @Override
    public void periodic() {
        if(Constants.getClimberArmStatus() == CLIMBER_ARM_STATUS.COLLISION) {
            /*
            It's very possible that the arm and climber are actively touching, and we don't know how, so its safest
            to just put the motors in break.
            */
            stop();
        } else if(Constants.getClimberArmStatus() == CLIMBER_ARM_STATUS.DANGER) {
            /*
            The climber is higher than we want, and getting close to touching the arm. Move the arm to the protected
            position, just to be safe.
            */
            ArmPosition.PROTECTED.goTo();
        }
    }

    public enum ArmPosition {
        GROUND(0.0),
        CLIMB(0.0),// We may use this position as somewhere above the ground to protect from bumper collision, but under the stage height
        PROTECTED(5.0),
        //START((Constants.getArmEncoder() - armSubsystem.ANALOG_ENCODER_ZERO) * gearRatio),
        SOURCE(19.539),
        AMP(31.5);

        // 90 = 24.9522
        //Analog = 0.6381

        private static final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

        public final double position;

        ArmPosition(double position) {
            this.position = position;
        }

        public void goTo() {
            ArmSubsystem.getInstance().goToSmartMotionPosition(this.position);
            requestedPosition = this.position;
        }

        public boolean isInPosition() {
            return armSubsystem.isInPosition(this.position);
        }
    }
}

