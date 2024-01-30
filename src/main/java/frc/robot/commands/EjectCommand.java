package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class EjectCommand extends Command {
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    private final double shooterEjectRpm = 1000;

    private int ticksElapsed;

   /**
    * {@value #isFinishedTicks} * 20ms = time in sec
    */
   private static final int isFinishedTicks = 100;

    public EjectCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.collectorSubsystem, this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        ticksElapsed = 0;
        collectorSubsystem.eject();
        shooterSubsystem.setVelocity(-shooterEjectRpm);
    }

    @Override
    public void execute() {
        ticksElapsed++;
    }

    @Override
    public boolean isFinished() {
        return ticksElapsed >= isFinishedTicks;
    }

    @Override
    public void end(boolean interrupted) {
        collectorSubsystem.stop();
        shooterSubsystem.stop();
    }
}
