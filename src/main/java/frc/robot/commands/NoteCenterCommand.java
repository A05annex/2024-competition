package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CollectorSubsystem;

import javax.swing.plaf.IconUIResource;


public class NoteCenterCommand extends Command {
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();

    public static boolean isCentered = true;

    private int centerTimer = 0;

    private PHASE phase;

    private enum PHASE {
       CENTER,
       REVERSE,
       FORWARD,
    }

    public NoteCenterCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.collectorSubsystem);
    }

    @Override
    public void initialize() {
        centerTimer = 0;
        if(Constants.NOTE_SENSOR.get()) {
          isCentered = true;
        } else {
            isCentered = false;
            phase = PHASE.CENTER;
            collectorSubsystem.setPower(0.2);
        }
    }

    @Override
    public void execute() {
        if(isCentered) {
          return;
       }

       if(phase == PHASE.CENTER) {
           centerTimer++;
           if(centerTimer > 45) {
               collectorSubsystem.setPower(-0.2);
               phase = PHASE.REVERSE;
               centerTimer = 0;
           }
       }
       else if(phase == PHASE.REVERSE) {
          if(Constants.NOTE_SENSOR.get()) { // Sensor cleared
              if(centerTimer > 5) {
                 phase = PHASE.FORWARD;
                 collectorSubsystem.setPower(0.1);
             }
             centerTimer++;
          }
       } else if(phase == PHASE.FORWARD) {
          if(!Constants.NOTE_SENSOR.get()) { // Sensor blocked
             isCentered = true;
             collectorSubsystem.stop();
          }
       }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
