package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class AmpScoreCommandGroup extends SequentialCommandGroup {
    public AmpScoreCommandGroup() {
        super(new AmpAprilTagCommand(Constants.CAMERA, 0.95, -0.06, "amp"), new AmpArmCommand());
    }
}