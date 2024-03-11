package frc.robot.commands;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class DynamicTargetRightCommandGroup extends ConditionalCommand {
    public DynamicTargetRightCommandGroup() {
        super(
                new ConditionalCommand(
                        new SequentialCommandGroup(new AprilTagPositionCommand(Constants.CAMERA, 0.1, 0.0, "source far")),
                        new ConditionalCommand(
                                new SequentialCommandGroup(new AprilTagPositionCommand(Constants.CAMERA, 0.1, 0.0, "source close")),
                                new SequentialCommandGroup(new AprilTagPositionCommand(Constants.CAMERA, 0.1, 0.5, "source far")),
                                DynamicTargetRightCommandGroup::isSourceRight),
                        DynamicTargetRightCommandGroup::isSourceLeft),
                new SequentialCommandGroup(new AprilTagPositionCommand(Constants.CAMERA, 0.33, 0.0, "amp")),
                DynamicTargetRightCommandGroup::isBlueAlliance);

        /*
        Using multiple levels of logic in the command composition style is not very readable, below is a logic flow tree (Left is true, right is false)


                                      DynamicTargetRightCommandGroup
                                                   |
                                   /--------isBlueAlliance--------\
                                  /                                \
                     /------isSourceLeft------\             RedAmpPlaceGroup
                    /                          \
           LeftSourcePlaceGroup      /-------isSourceRight-------\
                                    /                             \
                          RightSourcePlaceGroup           CenterSourcePlaceGroup
         */
    }

    private static boolean isBlueAlliance() {
        return !NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
    }

    private static boolean isSourceLeft() {
        return Constants.ALT_XBOX.getPOV() == 270;
    }

    private static boolean isSourceRight() {
        return Constants.ALT_XBOX.getPOV() == 90;
    }

    // new SequentialCommandGroup(new AprilTagPositionCommand(Constants.CAMERA, 1.0, 0.0, ))
}