package frc.robot.commands;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class DynamicTargetLeftCommandGroup extends ConditionalCommand {
    public DynamicTargetLeftCommandGroup() {
        //TODO: add commands to do appropriate arm moves
        super(
            new ConditionalCommand(
                new SequentialCommandGroup(new AprilTagPositionCommand(Constants.CAMERA, 0.1, 0.0, "source close")),
                new ConditionalCommand(
                    new SequentialCommandGroup(new AprilTagPositionCommand(Constants.CAMERA, 0.1, 0.0, "source far")),
                    new SequentialCommandGroup(new AprilTagPositionCommand(Constants.CAMERA, 0.1, 0.5, "source close")),
                    DynamicTargetLeftCommandGroup::isSourceRight),
                DynamicTargetLeftCommandGroup::isSourceLeft),
            new SequentialCommandGroup(new AprilTagPositionCommand(Constants.CAMERA, 0.33, 0.0, "amp")),
            DynamicTargetLeftCommandGroup::isRedAlliance);

        /*
        Using multiple levels of logic in the command composition style is not very readable, below is a logic flow tree (Left is true, right is false)


                                      DynamicTargetLeftCommandGroup
                                                   |
                                   /--------isRedAlliance--------\
                                  /                               \
                     /------isSourceLeft------\            BlueAmpPlaceGroup
                    /                          \
           LeftSourcePlaceGroup      /-------isSourceRight-------\
                                    /                             \
                          RightSourcePlaceGroup           CenterSourcePlaceGroup
         */
    }

    private static boolean isRedAlliance() {
        return NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
    }

    private static boolean isSourceLeft() {
        return Constants.ALT_XBOX.getPOV() == 270;
    }

    private static boolean isSourceRight() {
        return Constants.ALT_XBOX.getPOV() == 90;
    }

    // new SequentialCommandGroup(new AprilTagPositionCommand(Constants.CAMERA, 1.0, 0.0, ))
}