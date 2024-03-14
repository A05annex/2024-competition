package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.Utl;


public class ManualClimberCommand extends Command {
    private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();

    private double stick = -A05Constants.ALT_XBOX.getLeftY();
    private final double DEADBAND = 0.05;
    private boolean wasSpinning;

    private boolean climbPos = false;

    public ManualClimberCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climberSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        wasSpinning = false;
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        stick = -A05Constants.ALT_XBOX.getLeftY();


//        if(Constants.getClimberArmStatus() == Constants.CLIMBER_ARM_STATUS.COLLISION) {
//            /*
//            It's very possible that the arm and climber are actively touching, and we don't know how, so its safest
//            to just put the motors in brake mode.
//            */
//            climberSubsystem.stop();
//            return;
//        } else if(Constants.getClimberArmStatus() == Constants.CLIMBER_ARM_STATUS.DANGER) {
//            /*
//            The climber is higher than we want, and getting close to touching the arm. Try moving the climber to zero
//            */
//            climberSubsystem.goToSmartMotionPosition(0.0);
//            return;
//        }

        if(!Utl.inTolerance(stick, 0.0, DEADBAND)) {
            climberSubsystem.goToDeltaPosition(stick * 50.0); // Distance far enough that the motor will do smart motion accel
            wasSpinning = true;
        } else if(wasSpinning) {
            climberSubsystem.goToDeltaPosition(0.0);
            wasSpinning = false;
        }
    }

    // No isFinished() or end() because this command should only be canceled when another command is run
}
