package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmWristCommand extends SequentialCommandGroup {

    public ArmWristCommand(ArmCommand arm, WristCommand wrist, double minArmThreshold, double minWristThreshold, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> (wristSubsystem.getPosition() > minWristThreshold)),
                    arm
                ),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> (armSubsystem.getPosition() > minArmThreshold)),
                    wrist
                )
            )
        );
    }

}
