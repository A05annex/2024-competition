package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import org.a05annex.frc.A05Constants;
import org.a05annex.util.Utl;


public class ManualClimberCommand extends Command {
    private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();

    private final double stick = A05Constants.ALT_XBOX.getLeftY();

    private boolean wasSpinning;

    private final double DEADBAND = 0.05;

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
        if(!Utl.inTolerance(stick, 0.0, DEADBAND)) {
            climberSubsystem.goToDeltaPosition(stick * 10.0); // Distance far enough that the motor will do smart motion accel
            wasSpinning = true;
            return;
        }

        if(wasSpinning) {
            climberSubsystem.goToDeltaPosition(0.0);
            wasSpinning = false;
        }
    }

    // No isFinished() or end() because this command should only be canceled when another command is run
}
