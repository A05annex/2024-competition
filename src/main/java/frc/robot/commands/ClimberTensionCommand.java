package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberTensionCommand extends Command {
    private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();

    private int motorStartup = 0;

    private boolean leftStopped;
    private boolean rightStopped;

    public ClimberTensionCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climberSubsystem);
    }

    @Override
    public void initialize() {
//        if(climberSubsystem.isInitialized()) {
//            return;
//        }

        climberSubsystem.setLeftEncoderPosition(climberSubsystem.maxPos);
        climberSubsystem.setRightEncoderPosition(climberSubsystem.maxPos);

        double rpm = -300.0;

        climberSubsystem.leftMotorRpm(rpm);
        climberSubsystem.rightMotorRpm(rpm);

        motorStartup = 0;
    }

    @Override
    public void execute() {
        if(motorStartup < 20) {
            motorStartup++;
            return;
        }

        double stallRpm = -200.0;

        if(!leftStopped) {
            if(climberSubsystem.getLeftRpm() > stallRpm) {
                climberSubsystem.setLeftEncoderPosition(0.0);
                climberSubsystem.leftMotorSmartMotionPosition(0.0);
                leftStopped = true;
            }
        }

        if(!rightStopped) {
            if(climberSubsystem.getRightRpm() > stallRpm) {
                climberSubsystem.setRightEncoderPosition(0.0);
                climberSubsystem.rightMotorSmartMotionPosition(0.0);
                rightStopped = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return leftStopped && rightStopped;
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            climberSubsystem.initialize();
        }
    }
}
