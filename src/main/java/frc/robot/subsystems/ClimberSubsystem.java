package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.util.Utl;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkNeo rightMotor = SparkNeo.factory(Constants.CAN_Devices.RIGHT_CLIMBER_MOTOR);
    private final SparkNeo leftMotor = SparkNeo.factory(Constants.CAN_Devices.LEFT_CLIMBER_MOTOR);
    // Declare PID constants for smart motion control
    private final double smKp = 0.00005, smKi = 0.000, smKiZone = 0.0, smKff = 0.000156, smMaxRPM = 5000.0,
            smMaxDeltaRPMSec = 10000.0, smMinRPM = 0.0, smError = 0.1;
    // Declare PID constants for position control
    private final double posKp = 0.22, posKi = 0.0, posKiZone = 0.0, posKff = 0.0;
    // Declare PID constants for speed (rpm) control
    private final double rpmKp = 0.0, rpmKi = 0.0, rpmKiZone = 0.0, rpmKff = 0.00019;

    public final double maxPos = 180.0;

    // Declare min and max soft limits and where the motor thinks it starts
    private final Double minPosition = 0.0, maxPosition = 165.0, startPosition = 0.0;
    private final double inPositionTolerance = 0.5;
    private double offset = 0.0;
    private double leftReqPos = 0.0;
    private double rightReqPos = 0.0;
    private boolean initialized = false;

    private ClimberSubsystem() {
        rightMotor.startConfig();
        rightMotor.setCurrentLimit(SparkNeo.UseType.RPM_OCCASIONAL_STALL, SparkNeo.BreakerAmps.Amps40);
        rightMotor.setSoftLimits(minPosition, maxPosition);
        rightMotor.setDirection(SparkNeo.Direction.REVERSE);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotor.setPositionPID(posKp, posKi, posKiZone, posKff);
        rightMotor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        rightMotor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        rightMotor.endConfig();
        rightMotor.setEncoderPosition(0.0);

        leftMotor.startConfig();
        leftMotor.setCurrentLimit(SparkNeo.UseType.RPM_OCCASIONAL_STALL, SparkNeo.BreakerAmps.Amps40);
        leftMotor.setSoftLimits(minPosition, maxPosition);
        leftMotor.setDirection(SparkNeo.Direction.DEFAULT);
        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMotor.setPositionPID(posKp, posKi, posKiZone, posKff);
        leftMotor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        leftMotor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        leftMotor.endConfig();
        leftMotor.setEncoderPosition(0.0);

        System.out.println("****************** CLIMBER CONSTRUCTED **********************");
        System.out.flush();

        initialized = false;
    }

    private final static ClimberSubsystem INSTANCE = new ClimberSubsystem();
    public static ClimberSubsystem getInstance() {
        return INSTANCE;
    }

    public void goToSmartMotionPosition(double position) {
        rightMotor.setSmartMotionTarget(position + offset);
        leftMotor.setSmartMotionTarget(position - offset);
        leftReqPos = position;
        rightReqPos = position;
    }

    public void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    public double getRightPosition() {
        return rightMotor.getEncoderPosition();
    }

    public double getRightRpm() {
        return rightMotor.getEncoderVelocity();
    }

    public double getLeftPosition() {
        return leftMotor.getEncoderPosition();
    }

    public double getLeftRpm() {
        return leftMotor.getEncoderVelocity();
    }

    public void goToDeltaPosition(double delta) {
        goToSmartMotionPosition(getRightPosition() + delta);
    }

    public void leftMotorSmartMotionPosition(double position) {
        leftMotor.setSmartMotionTarget(position);
        leftReqPos = position;
    }

    public void leftMotorRpm(double rpm) {
        leftMotor.setTargetRPM(rpm);
    }

    public void rightMotorSmartMotionPosition(double position) {
        rightMotor.setSmartMotionTarget(position);
        rightReqPos = position;
    }

    public void rightMotorRpm(double rpm) {
        rightMotor.setTargetRPM(rpm);
    }

    public void stopLeftMotor() {
        leftMotor.stopMotor();
    }

    public void stopRightMotor() {
        rightMotor.stopMotor();
    }

    public void setLeftEncoderPosition(double pos) {
        leftMotor.setEncoderPosition(pos);
    }

    public void setRightEncoderPosition(double pos) {
        rightMotor.setEncoderPosition(pos);
    }

    public boolean isInPosition(double position) {
        return Utl.inTolerance(leftMotor.getEncoderPosition(), position, inPositionTolerance) && Utl.inTolerance(rightMotor.getEncoderPosition(), position, inPositionTolerance);
    }

    public boolean isInPosition() {
        return Utl.inTolerance(leftMotor.getEncoderPosition(), leftReqPos, inPositionTolerance) && Utl.inTolerance(rightMotor.getEncoderPosition(), rightReqPos, inPositionTolerance);
    }

    public boolean isInitialized() {
        return initialized;
    }

    public void initialize() {
        initialized = true;
    }
}

