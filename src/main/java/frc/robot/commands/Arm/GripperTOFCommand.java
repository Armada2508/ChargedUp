package frc.robot.commands.Arm;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.GripperSubsystem;

public class GripperTOFCommand extends SequentialCommandGroup {

    public GripperTOFCommand(double gripperPosition, double distanceCentimeters, TimeOfFlight tof, GripperSubsystem gripperSubsystem) {
        addCommands(
            new WaitUntilCommand(() -> {
                if (tof.isRangeValid()) {
                    return (tof.getRange() * 10) < distanceCentimeters;
                }
                return false;
            }),
            new GripperCommand(gripperPosition, gripperSubsystem)
        );
    }

}
