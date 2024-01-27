package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.util.Utl;

public class ArmSubsystem extends SubsystemBase {

    private final SparkNeo forwardMotor = SparkNeo.factory(Constants.CAN_Devices.FORWARD_MOTOR);
    private final SparkNeo backwardMotor = SparkNeo.factory(Constants.CAN_Devices.BACKWARD_MOTOR);

    // Declare PID constants for smart motion control
    private final double smKp = 0.00005, smKi = 0.000, smKiZone = 0.0, smKff = 0.000156, smMaxRPM = 3000.0,
            smMaxDeltaRPMSec = 3000.0, smMinRPM = 0.0, smError = 0.1;

    // Declare PID constants for position control
    private final double posKp = 0.22, posKi = 0.0, posKiZone = 0.0, posKff = 0.0;

    // Declare PID constants for speed (rpm) control
    private final double rpmKp = 0.5, rpmKi = 0.0, rpmKiZone = 0.0, rpmKff = 0.0;

    // Declare min and max soft limits and where the motor thinks it starts
    private final Double minPosition = null, maxPosition = 5000.0, startPosition = 100.0;

    private final double IN_POSITION_DEADBAND = 0.5;

    private static double requestedPosition = getInstance().startPosition;

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
        forwardMotor.setEncoderPosition(startPosition);

        backwardMotor.startConfig();
        backwardMotor.setCurrentLimit(SparkNeo.UseType.POSITION, SparkNeo.BreakerAmps.Amps40);
        backwardMotor.setSoftLimits(minPosition, maxPosition);
        backwardMotor.setDirection(SparkNeo.Direction.REVERSE);
        backwardMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        backwardMotor.setPositionPID(posKp, posKi, posKiZone, posKff);
        backwardMotor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        backwardMotor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        backwardMotor.endConfig();
        backwardMotor.setEncoderPosition(startPosition);
    }

    //public void double backwardMotor.SampleMotorSubsystem null (0.0);:

    public enum ArmPosition {
        GROUND(0.0),
        START(getInstance().startPosition),
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



    public void goToSmartMotionPosition(double position) {
        forwardMotor.setSmartMotionTarget(position);
        backwardMotor.setSmartMotionTarget(position);
    }

    public void goToPosition(double position) {
        forwardMotor.setTargetPosition(position);
        backwardMotor.setTargetPosition(position);
    }

    public void stop() {
        forwardMotor.stopMotor();
        backwardMotor.stopMotor();
    }

    public double getPosition() {
        return forwardMotor.getEncoderPosition();
    }

    public boolean isInPosition() {
        return Utl.inTolerance(forwardMotor.getEncoderPosition(), requestedPosition, IN_POSITION_DEADBAND) &&
                Utl.inTolerance(backwardMotor.getEncoderPosition(), requestedPosition, IN_POSITION_DEADBAND);
    }

    public boolean isInPosition(double position) {
        return Utl.inTolerance(forwardMotor.getEncoderPosition(), position, IN_POSITION_DEADBAND) &&
                Utl.inTolerance(backwardMotor.getEncoderPosition(), position, IN_POSITION_DEADBAND);
    }
}

