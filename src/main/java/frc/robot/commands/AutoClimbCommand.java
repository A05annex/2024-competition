package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import org.a05annex.frc.NavX;
import org.a05annex.util.Utl;


public class AutoClimbCommand extends Command {
    private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();

    private final double angleThreshold = 2.0;
    private final double settleTolerance = 0.01;

    private double lastRoll = 0.0;

    enum CLIMBER_PHASE {
        RAISING,
        LEFT_TENSION,
        RIGHT_TENSION,
        SETTLE,
        LIFT
    }

    private CLIMBER_PHASE phase = CLIMBER_PHASE.RAISING;

    public AutoClimbCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climberSubsystem, this.armSubsystem);
    }

    @Override
    public void initialize() {
        phase = CLIMBER_PHASE.RAISING;
        climberSubsystem.goToSmartMotionPosition(climberSubsystem.maxPos);
        lastRoll = 0.0;
        ArmSubsystem.ArmPosition.CLIMB.goTo();
    }

    @Override
    public void execute() {
        // The arm is not where we want it, return early
        if(!armSubsystem.isInPosition(ArmSubsystem.ArmPosition.CLIMB.position)) {
            return;
        }

        switch(phase) {
            case RAISING:
                if(climberSubsystem.isInPosition()) {
                    phase = CLIMBER_PHASE.LEFT_TENSION;
                    climberSubsystem.leftMotorSmartMotionPosition(0.0);
                }
                break;
            case LEFT_TENSION:
                if(NavX.getInstance().getNavInfo().pitch.getDegrees() > angleThreshold) {
                    phase = CLIMBER_PHASE.RIGHT_TENSION;
                    climberSubsystem.stopLeftMotor();
                    climberSubsystem.rightMotorSmartMotionPosition(0.0);
                }
                break;
            case RIGHT_TENSION:
                if(NavX.getInstance().getNavInfo().pitch.getDegrees() < angleThreshold) {
                    phase = CLIMBER_PHASE.SETTLE;
                    climberSubsystem.stopRightMotor();
                    lastRoll = NavX.getInstance().getNavInfo().pitch.getDegrees();
                }
                break;
            case SETTLE:
                if(Utl.inTolerance(NavX.getInstance().getNavInfo().pitch.getDegrees(), lastRoll, settleTolerance)) {
                    phase = CLIMBER_PHASE.LIFT;
                    double posDelta = climberSubsystem.getLeftPosition() - climberSubsystem.getRightPosition();

                    //is left arm higher than right?
                    if(posDelta >= 0.0) {
                        climberSubsystem.leftMotorSmartMotionPosition(posDelta);
                        climberSubsystem.rightMotorSmartMotionPosition(0.0);
                    } else {
                        climberSubsystem.leftMotorSmartMotionPosition(0.0);
                        climberSubsystem.rightMotorSmartMotionPosition(posDelta);
                    }
                }
                lastRoll = NavX.getInstance().getNavInfo().pitch.getDegrees();
                break;

        }
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.isInPosition();
    }

    @Override
    public void end(boolean interrupted) {

    }
}
