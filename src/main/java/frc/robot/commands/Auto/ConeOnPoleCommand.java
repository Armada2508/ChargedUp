package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Wrist;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.ArmWristCommand;
import frc.robot.commands.arm.WristCommand;
import frc.robot.commands.auto.PlacePieceCommand.Height;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ConeOnPoleCommand extends SequentialCommandGroup {

    public ConeOnPoleCommand(Height height, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        double arm = 0, wrist = Wrist.maxDegrees;
        if (height == Height.MID) {
            arm = 78; 
            wrist = 45; 
        } else {
            arm = 100;
            wrist = 25;
        }
        addCommands(
            new ArmWristCommand(new ArmCommand(arm, 45, 45, armSubsystem), new WristCommand(wrist, 130, 130, wristSubsystem, armSubsystem), 30, -15, armSubsystem, wristSubsystem, gripperSubsystem)
        );
    }

}
