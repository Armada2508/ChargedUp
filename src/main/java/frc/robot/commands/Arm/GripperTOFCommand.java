package frc.robot.commands.arm;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Gripper;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class GripperTOFCommand extends SequentialCommandGroup {

    public GripperTOFCommand(double distanceCentimeters, TimeOfFlight tof, GripperSubsystem gripperSubsystem, ArmSubsystem armSubsystem) {
        addCommands(
            new WaitUntilCommand(() -> {
                if (tof.isRangeValid()) {
                    return (tof.getRange() * 10) < distanceCentimeters;
                }
                return false;
            }),
            new GripperCommand(Gripper.grabCone, gripperSubsystem, armSubsystem)
        );
    }

}
