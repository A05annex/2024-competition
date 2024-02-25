package frc.robot.subsystems;


import  com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.util.Utl;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkNeo leftMotor = SparkNeo.factory(Constants.CAN_Devices.LEFT_SHOOTER_MOTOR);
    private final SparkNeo rightMotor = SparkNeo.factory(Constants.CAN_Devices.RIGHT_SHOOTER_MOTOR);

    // Declare PID constants for smart motion control
    private final double smKp = 0.00005, smKi = 0.000, smKiZone = 0.0, smKff = 0.000156, smMaxRPM = 3000.0,
            smMaxDeltaRPMSec = 3000.0, smMinRPM = 0.0, smError = 0.1;

    // Declare PID constants for position control
    private final double posKp = 0.22, posKi = 0.0, posKiZone = 0.0, posKff = 0.0;

    // Declare PID constants for speed (rpm) control
    private final double rpmKp = 0.0006, rpmKi = 0.000001, rpmKiZone = 200.0, rpmKff = 0.000156;

    // Declare min and max soft limits and where the leftMotor thinks it starts
    private final Double minPosition = null, maxPosition = null, startPosition = 0.0;

    private double requestedRpm;

    private final static ShooterSubsystem INSTANCE = new ShooterSubsystem();
    public static ShooterSubsystem getInstance() {
        return INSTANCE;
    }

    private ShooterSubsystem() {
        leftMotor.startConfig();
        leftMotor.setCurrentLimit(SparkNeo.UseType.FREE_SPINNING, SparkNeo.BreakerAmps.Amps40);
        leftMotor.setSoftLimits(minPosition, maxPosition);
        leftMotor.setDirection(SparkNeo.Direction.REVERSE);
        leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftMotor.setPositionPID(posKp, posKi, posKiZone, posKff);
        leftMotor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        leftMotor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        leftMotor.endConfig();
        leftMotor.setEncoderPosition(startPosition);

        rightMotor.startConfig();
        rightMotor.setCurrentLimit(SparkNeo.UseType.FREE_SPINNING, SparkNeo.BreakerAmps.Amps40);
        rightMotor.setSoftLimits(minPosition, maxPosition);
        rightMotor.setDirection(SparkNeo.Direction.DEFAULT);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightMotor.setPositionPID(posKp, posKi, posKiZone, posKff);
        rightMotor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        rightMotor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        rightMotor.endConfig();
        rightMotor.setEncoderPosition(startPosition);

        requestedRpm = 0;
    }


    public void setVelocity(double rpm) {
        leftMotor.setTargetRPM(rpm);
        rightMotor.setTargetRPM(rpm);
        requestedRpm = rpm;
    }

    public void setLinearInterpolation(Constants.LinearInterpolation linearInterpolation) {
        setVelocity(linearInterpolation.rpm);
    }

    public boolean isAtRpm() {
        return isAtRpm(requestedRpm);
    }

    public boolean isAtRpm(double rpm) {
        return Utl.inTolerance(getVelocity(), rpm, 100.0);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();

        requestedRpm = 0.0;
    }

    public void shoot() {
        setVelocity(5000.0);
    }

    public double getVelocity() {
        return leftMotor.getEncoderVelocity();
    }
}

