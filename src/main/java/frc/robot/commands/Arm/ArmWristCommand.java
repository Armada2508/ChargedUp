package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Gripper;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmWristCommand extends SequentialCommandGroup {

    private final double minWristDegrees = 80; // minimum degrees the wrist can be at to move the arm inside the frame
    
    public ArmWristCommand(ArmCommand arm, WristCommand wrist, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new ConditionalCommand(gripper(gripperSubsystem), Commands.none(), armSubsystem::isInsideFrame),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> (wristSubsystem.getPosition() > minWristDegrees || arm.getTarget() > Arm.insideFrameDeg)),
                    arm
                ),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> (armSubsystem.getPosition() > Arm.insideFrameDeg || wrist.getTarget() > minWristDegrees)),
                    wrist
                )
            )
        );
    }

    private Command gripper(GripperSubsystem gripperSubsystem) {
        return new ConditionalCommand(new GripperCommand(Gripper.closed, gripperSubsystem), Commands.none(), () -> gripperSubsystem.getPhysicalPosition() <= Gripper.closed);
    }

}
