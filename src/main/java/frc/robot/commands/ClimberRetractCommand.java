package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberRetractCommand extends Command {
    private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    // private final ArmPositions armSubsystem = ArmSubsystem.ArmPositions;

    public ClimberRetractCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)


        //addRequirements(this.climberSubsystem, ArmSubsystem.getInstance());

        addRequirements(this.climberSubsystem);
    }

    @Override
    public void initialize() {
        ArmSubsystem.ArmPosition.CLIMB.goTo();

        climberSubsystem.goToSmartMotionPosition(0.0);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {

        // return Utl.inTolerance(climberSubsystem.getPosition(), 0.0, 0.5) && armSubsystem.climb.isInPosition();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
