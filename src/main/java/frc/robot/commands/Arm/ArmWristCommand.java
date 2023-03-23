package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Gripper;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmWristCommand extends SequentialCommandGroup {

    private final double minArmDegrees = 10;
    private final double minWristDegrees = 80;
    
    public ArmWristCommand(ArmCommand arm, WristCommand wrist, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new ConditionalCommand(new GripperCommand(Gripper.almostClosed, gripperSubsystem), Commands.none(), () -> gripperSubsystem.getPosition() <= Gripper.almostClosed),
            new ConditionalCommand(
                armOut(arm, wrist, armSubsystem), /* Going Outside Frame */
                armIn(arm, wrist, wristSubsystem), /* Going Inside Frame */
                () -> arm.getTarget() > minArmDegrees
            )
        );
    }

    private Command armOut(ArmCommand armCommand, WristCommand wristCommand, ArmSubsystem armSubsystem) {
        return new ParallelCommandGroup(
            armCommand,
            new SequentialCommandGroup(
                    new WaitUntilCommand(() -> armSubsystem.getPosition() > minArmDegrees),
                    wristCommand
            )
        );
    }

    private Command armIn(ArmCommand armCommand, WristCommand wristCommand, WristSubsystem wristSubsystem) {
        return new ParallelCommandGroup(
            wristCommand,
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> wristSubsystem.getPosition() > minWristDegrees),
                armCommand
            )
        );
    }

}
