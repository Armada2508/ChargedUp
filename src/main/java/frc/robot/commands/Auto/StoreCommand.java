package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Gripper;
import frc.robot.Constants.Wrist;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.ArmWristCommand;
import frc.robot.commands.arm.GripperCommand;
import frc.robot.commands.arm.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class StoreCommand extends SequentialCommandGroup {
    
    public StoreCommand(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new GripperCommand(Gripper.onLimit, gripperSubsystem, armSubsystem),
            new ArmWristCommand(new ArmCommand(Arm.minDegrees, 90, 60, armSubsystem), new WristCommand(Wrist.maxDegrees, 120, 100, wristSubsystem, armSubsystem), -1, 100, armSubsystem, wristSubsystem, gripperSubsystem)
        );
    }

}
