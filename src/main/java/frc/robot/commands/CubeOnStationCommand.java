package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class CubeOnStationCommand extends CommandBase {

    private SequentialCommandGroup group;

    public CubeOnStationCommand(Height height, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        group = new SequentialCommandGroup(
            
        );
    }

    public Command getCommand() {
        return group;
    }

    public enum Height {
        MID,
        TALL
    }

}
