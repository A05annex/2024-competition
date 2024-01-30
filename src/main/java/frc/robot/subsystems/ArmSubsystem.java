package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.util.Utl;

public class ArmSubsystem extends SubsystemBase {

    private final SparkNeo forwardMotor = SparkNeo.factory(Constants.CAN_Devices.FORWARD_ARM_MOTOR);
    private final SparkNeo backwardMotor = SparkNeo.factory(Constants.CAN_Devices.BACKWARD_ARM_MOTOR);

    // Declare PID constants for smart motion control
    private final double smKp = 0.00005, smKi = 0.000, smKiZone = 0.0, smKff = 0.000156, smMaxRPM = 3000.0,
            smMaxDeltaRPMSec = 3000.0, smMinRPM = 0.0, smError = 0.1;

    // Declare PID constants for position control
    private final double posKp = 0.22, posKi = 0.0, posKiZone = 0.0, posKff = 0.0;

    // Declare PID constants for speed (rpm) control
    private final double rpmKp = 0.5, rpmKi = 0.0, rpmKiZone = 0.0, rpmKff = 0.0;

    // Declare min and max soft limits and where the motor thinks it starts
    private final Double minPosition = null, maxPosition = 5000.0;

    // Tolerance to decide if in position
    private final double IN_POSITION_DEADBAND = 0.5;

    // Position most recently requested of the arm
    private static double requestedPosition = ArmPosition.START.position;

    private boolean enableInit = false;

    private final static ArmSubsystem INSTANCE = new ArmSubsystem();
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

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
        forwardMotor.setEncoderPosition(ArmPosition.START.position);

        backwardMotor.startConfig();
        backwardMotor.setCurrentLimit(SparkNeo.UseType.POSITION, SparkNeo.BreakerAmps.Amps40);
        backwardMotor.setSoftLimits(minPosition, maxPosition);
        backwardMotor.setDirection(SparkNeo.Direction.REVERSE);
        backwardMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        backwardMotor.setPositionPID(posKp, posKi, posKiZone, posKff);
        backwardMotor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        backwardMotor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        backwardMotor.endConfig();
        backwardMotor.setEncoderPosition(ArmPosition.START.position);
    }

    //public void double backwardMotor.SampleMotorSubsystem null (0.0);:

    public enum ArmPosition {
        GROUND(0.0),
        START(100),
        SOURCE(200.0),
        AMP(300.0);

        private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

        public final double position;

        ArmPosition(double position) {
            this.position = position;
        }

        public void goTo() {
            armSubsystem.goToSmartMotionPosition(this.position);
            requestedPosition = this.position;
        }

        public boolean isInPosition() {
            return armSubsystem.isInPosition(this.position);
        }
    }


    public void enableInit() {
        if(enableInit) {
            return;
        }

        double startTime = Timer.getFPGATimestamp();
        System.out.println("****************************************************************");
        System.out.println("ENABLE INIT STARTED: " + startTime);
        System.out.println("****************************************************************");

        forwardMotor.setEncoderPosition(ArmPosition.START.position);
        backwardMotor.setEncoderPosition(ArmPosition.START.position);

        // Lock the forward (supporting) motor to start pos.
        forwardMotor.setTargetPosition(ArmPosition.START.position);
        System.out.println("****************************************************************");
        System.out.printf("TIME: %f; support = %f; tension = %f%n",
                Timer.getFPGATimestamp() - startTime, forwardMotor.getEncoderPosition(), backwardMotor.getEncoderPosition());

        // 0.5 amps, just enough to tension
        backwardMotor.sparkMaxPID.setReference(1.2, CANSparkMax.ControlType.kVoltage);

        while(true) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                continue;
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
        backwardMotor.setEncoderPosition(forwardMotor.getEncoderPosition());

        System.out.printf("END TIME: %f; support = %f; tension = %f%n",
                Timer.getFPGATimestamp() - startTime, forwardMotor.getEncoderPosition(), backwardMotor.getEncoderPosition());
        System.out.println("****************************************************************");
        System.out.println("****************************************************************");
        System.out.println("****************************************************************");

        // Hold the arms at the start position
        forwardMotor.setSmartMotionTarget(ArmPosition.START.position);
        forwardMotor.setSmartMotionTarget(ArmPosition.START.position);

        requestedPosition = ArmPosition.START.position;
        enableInit = true;
    }


    // Go to position with smart motion
    public void goToSmartMotionPosition(double position) {
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

    // Returns if the arm is in the position the most recent requested position
    public boolean isInPosition() {
        return Utl.inTolerance(forwardMotor.getEncoderPosition(), requestedPosition, IN_POSITION_DEADBAND) &&
                Utl.inTolerance(backwardMotor.getEncoderPosition(), requestedPosition, IN_POSITION_DEADBAND);
    }

    // Returns if the arm is in the position passed in
    public boolean isInPosition(double position) {
        return Utl.inTolerance(forwardMotor.getEncoderPosition(), position, IN_POSITION_DEADBAND) &&
                Utl.inTolerance(backwardMotor.getEncoderPosition(), position, IN_POSITION_DEADBAND);
    }
}

