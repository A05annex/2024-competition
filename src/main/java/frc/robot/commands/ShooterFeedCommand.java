package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CollectorSubsystem;


public class ShooterFeedCommand extends Command {
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    private final int endTicks;
    private int currentTicks;
    private int noteTimeout = 0;

    public ShooterFeedCommand(int endTicks) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.collectorSubsystem);
        this.endTicks = endTicks;
    }

    public ShooterFeedCommand() {
        this(150); // Default if no value passed in is 3 seconds
    }

    @Override
    public void initialize() {
        collectorSubsystem.feed();
        currentTicks = 0;
        noteTimeout = 0;
    }

    @Override
    public void execute() {
        currentTicks++;
        System.out.println(currentTicks);

        if(Constants.NOTE_SENSOR.get()) {
            noteTimeout++;
        } else {
            noteTimeout = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return currentTicks >= endTicks || noteTimeout > 5;
    }

    @Override
    public void end(boolean interrupted) {
        collectorSubsystem.stop();
    }
}
