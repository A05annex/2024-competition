package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.util.Utl;

public class ClimberSubsystem extends SubsystemBase {

    private final static ClimberSubsystem INSTANCE = new ClimberSubsystem();
    public final double maxPos = 1000.0;
    private final SparkNeo rightMotor = SparkNeo.factory(Constants.CAN_Devices.RIGHT_CLIMBER_MOTOR);
    private final SparkNeo leftMotor = SparkNeo.factory(Constants.CAN_Devices.LEFT_SHOOTER_MOTOR);
    // Declare PID constants for smart motion control
    private final double smKp = 0.00005, smKi = 0.000, smKiZone = 0.0, smKff = 0.000156, smMaxRPM = 3000.0,
            smMaxDeltaRPMSec = 3000.0, smMinRPM = 0.0, smError = 0.1;
    // Declare PID constants for position control
    private final double posKp = 0.22, posKi = 0.0, posKiZone = 0.0, posKff = 0.0;
    // Declare PID constants for speed (rpm) control
    private final double rpmKp = 0.5, rpmKi = 0.0, rpmKiZone = 0.0, rpmKff = 0.0;
    // Declare min and max soft limits and where the motor thinks it starts
    private final Double minPosition = 0.0, maxPosition = maxPos, startPosition = 0.0;
    private final double inPositionTolerance = 0.5;
    private double offset = 0.0;
    private double leftReqPos = 0.0;
    private double rightReqPos = 0.0;
    private boolean enableInit = false;
    private int enableInitStartup = 0;

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
        rightMotor.setEncoderPosition(startPosition);

        leftMotor.startConfig();
        leftMotor.setCurrentLimit(SparkNeo.UseType.RPM_OCCASIONAL_STALL, SparkNeo.BreakerAmps.Amps40);
        leftMotor.setSoftLimits(minPosition, maxPosition);
        leftMotor.setDirection(SparkNeo.Direction.REVERSE);
        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMotor.setPositionPID(posKp, posKi, posKiZone, posKff);
        leftMotor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        leftMotor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        leftMotor.endConfig();
        leftMotor.setEncoderPosition(startPosition);

        enableInit = false;
        enableInitStartup = 0;
    }

    public static ClimberSubsystem getInstance() {
        return INSTANCE;
    }

    public void enableInit() {
        // Have we already run enableInit()
        if(enableInit) {
            // Yes? let's not do it again
            return;
        }

        enableInitStartup = 0;

        double rpm = -2500.0;

        leftMotor.setTargetRPM(rpm);
        rightMotor.setTargetRPM(rpm);
    }

    public void goToSmartMotionPosition(double position) {
        rightMotor.setSmartMotionTarget(position + offset);
        leftMotor.setSmartMotionTarget(position - offset);
        leftReqPos = position;
        rightReqPos = position;
    }

    public void resetEncoder() {
        rightMotor.setEncoderPosition(0.0);
    }

    public void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    public double getRightPosition() {
        return rightMotor.getEncoderPosition();
    }

    public double getLeftPosition() {
        return leftMotor.getEncoderPosition();
    }

    public void goToDeltaPosition(double delta) {
        goToSmartMotionPosition(getRightPosition() + delta);
    }

    public void leftMotorSmartMotionPosition(double position) {
        leftMotor.setSmartMotionTarget(position);
        leftReqPos = position;
    }

    public void rightMotorSmartMotionPosition(double position) {
        rightMotor.setSmartMotionTarget(position);
        rightReqPos = position;
    }

    public void stopLeftMotor() {
        leftMotor.stopMotor();
    }

    public void stopRightMotor() {
        rightMotor.stopMotor();
    }

    public boolean isInPosition(double position) {
        return Utl.inTolerance(leftMotor.getEncoderPosition(), position, inPositionTolerance) && Utl.inTolerance(rightMotor.getEncoderPosition(), position, inPositionTolerance);
    }

    public boolean isInPosition() {
        return Utl.inTolerance(leftMotor.getEncoderPosition(), leftReqPos, inPositionTolerance) && Utl.inTolerance(rightMotor.getEncoderPosition(), rightReqPos, inPositionTolerance);
    }

    @Override
    public void periodic() {
        if(enableInit || enableInitStartup < 5) {
            enableInitStartup++;
            return;
        }

        double stallRpm = -2000.0;

        if(leftMotor.getEncoderVelocity() < stallRpm) {
            leftMotor.setEncoderPosition(startPosition);
            leftMotorSmartMotionPosition(startPosition);
        }

        if(rightMotor.getEncoderVelocity() < stallRpm) {
            rightMotor.setEncoderPosition(startPosition);
            rightMotorSmartMotionPosition(startPosition);
        }
    }
}

