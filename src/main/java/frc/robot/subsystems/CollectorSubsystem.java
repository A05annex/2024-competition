package frc.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;

public class CollectorSubsystem extends SubsystemBase {

    private final SparkNeo motor = SparkNeo.factory(Constants.CAN_Devices.COLLECTOR_MOTOR);

    // Declare PID constants for smart motion control
    private final double smKp = 0.015, smKi = 0.000, smKiZone = 0.0, smKff = 0.000156, smMaxRPM = 3000.0,
            smMaxDeltaRPMSec = 3000.0, smMinRPM = 0.0, smError = 0.1;

    // Declare PID constants for position control
    private final double posKp = 0.22, posKi = 0.0, posKiZone = 0.0, posKff = 0.0;

    // Declare PID constants for speed (rpm) control
    private final double rpmKp = 0.0004, rpmKi = 0.000001, rpmKiZone = 200.0, rpmKff = 0.000156;

    // Declare min and max soft limits and where the motor thinks it starts
    private final Double minPosition = null, maxPosition = null, startPosition = 500.0;

    private final double intakeRPM = 5000, ejectRPM = 2000, feedRPM = 3000;

    private final static CollectorSubsystem INSTANCE = new CollectorSubsystem();
    public static CollectorSubsystem getInstance() {
        return INSTANCE;
    }

    private CollectorSubsystem() {
        motor.startConfig();
        motor.setCurrentLimit(SparkNeo.UseType.RPM_OCCASIONAL_STALL, SparkNeo.BreakerAmps.Amps40);
        motor.setSoftLimits(minPosition, maxPosition);
        motor.setDirection(SparkNeo.Direction.REVERSE);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.setPositionPID(posKp, posKi, posKiZone, posKff);
        motor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        motor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        motor.endConfig();
        motor.setEncoderPosition(startPosition);
    }

    public void goToSmartMotionPosition(double position) {
        motor.setSmartMotionTarget(position);
    }

    public void goToPosition(double position) {
        motor.setTargetPosition(position);
    }

    public void setVelocity(double rpm) {
        motor.setTargetRPM(rpm);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void intake() {
        setVelocity(intakeRPM);
    }

    public void eject() {
        setVelocity(-ejectRPM);
    }

    public void feed() {
        setVelocity(feedRPM);
    }

    public double getRpm() {
        return motor.getEncoderVelocity();
    }
}

