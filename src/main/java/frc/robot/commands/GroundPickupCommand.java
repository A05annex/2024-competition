package frc.robot.commands;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class GroundPickupCommand extends Command {
    private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();


    public GroundPickupCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.armSubsystem, this.collectorSubsystem, this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setBrakeMode(CANSparkBase.IdleMode.kBrake);
        ArmSubsystem.ArmPosition.GROUND.goTo();
        collectorSubsystem.intake();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return !Constants.NOTE_SENSOR.get(); // !.get() because true = clear.
        //return false;
    }

    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.ArmPosition.PROTECTED.goTo();
        collectorSubsystem.stop();
        shooterSubsystem.setBrakeMode(CANSparkBase.IdleMode.kCoast);
    }
}
