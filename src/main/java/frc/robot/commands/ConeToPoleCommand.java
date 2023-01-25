package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ConeToPoleCommand {

    private SequentialCommandGroup group;

    ConeToPoleCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        group = new SequentialCommandGroup(
            
        );
        group.schedule();
    }

}
