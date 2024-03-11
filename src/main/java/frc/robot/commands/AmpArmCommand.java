package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AmpArmCommand extends Command {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final ShooterFeedCommand shooterFeedCommand = new ShooterFeedCommand(150);
    private int isFinishedTimeout = -1;

    public AmpArmCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armSubsystem, this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        ArmSubsystem.ArmPosition.AMP.goTo();
        shooterSubsystem.setVelocity(4000);
        isFinishedTimeout = -1;
    }

    @Override
    public void execute() {
        if(ArmSubsystem.ArmPosition.AMP.isInPosition() && shooterSubsystem.isAtRpm()) {
            if(isFinishedTimeout == -1) {
                shooterFeedCommand.schedule();
            }
            isFinishedTimeout++;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinishedTimeout > 50 && shooterFeedCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.ArmPosition.PROTECTED.goTo();
        shooterSubsystem.stop();
    }
}
