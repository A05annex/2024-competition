package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectorSubsystem;


public class SourceIntakeCommand extends Command {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    public SourceIntakeCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armSubsystem, this.collectorSubsystem);
    }

    @Override
    public void initialize() {
        ArmSubsystem.ArmPosition.SOURCE.goTo();
    }

    @Override
    public void execute() {
        if(ArmSubsystem.ArmPosition.SOURCE.isInPosition() && !(collectorSubsystem.getRpm() > 500)) {
            collectorSubsystem.intake();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return !Constants.NOTE_SENSOR.get();
    }

    @Override
    public void end(boolean interrupted) {
        //ArmSubsystem.ArmPosition.PROTECTED.goTo();
        CollectorSubsystem.getInstance().stop();
    }
}
