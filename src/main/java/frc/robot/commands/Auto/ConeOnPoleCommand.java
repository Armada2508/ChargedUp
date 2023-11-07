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

    public static final int armHigh = 102;
    public static final int wristHigh = 45;

    public ConeOnPoleCommand(Height height, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        double arm = 0, wrist = Wrist.maxDegrees;
        if (height == Height.MID) {
            arm = 83; 
            wrist = 54; 
        } else {
            arm = armHigh;
            wrist = wristHigh;
        }
        addCommands(
            new ArmWristCommand(new ArmCommand(arm, 140, 120, armSubsystem), new WristCommand(wrist, 130, 130, wristSubsystem, armSubsystem), 30, -15, armSubsystem, wristSubsystem, gripperSubsystem)
        );
    }

}
