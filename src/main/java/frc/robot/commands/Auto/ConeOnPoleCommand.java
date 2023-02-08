package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Wrist;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.GripperCommand;
import frc.robot.commands.Arm.WristCommand;
import frc.robot.commands.Auto.PieceOnTopCommand.Height;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ConeOnPoleCommand extends SequentialCommandGroup {

    private final int initialDistance = 6;

    public ConeOnPoleCommand(Height height, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        if (height == Height.MID) addCommands(
            new WristCommand(Wrist.maxDegrees, wristSubsystem), // Don't hit pole
            new ArmCommand(80, armSubsystem),
            new AutoDriveCommand(initialDistance, driveSubsystem),
            new WristCommand(0, wristSubsystem),
            new GripperCommand(0, gripperSubsystem),
            new WaitCommand(1),
            // Reverse
            new WristCommand(Wrist.maxDegrees, wristSubsystem),
            new AutoDriveCommand(-initialDistance, driveSubsystem),
            new ArmCommand(Arm.minDegrees, armSubsystem)
        );
        else addCommands(
            new WristCommand(Wrist.maxDegrees, wristSubsystem), // Don't hit pole
            new ArmCommand(90, armSubsystem),
            new AutoDriveCommand(initialDistance, driveSubsystem),
            new WristCommand(0, wristSubsystem),
            new GripperCommand(0, gripperSubsystem),
            new WaitCommand(1),
            // Reverse
            new WristCommand(Wrist.maxDegrees, wristSubsystem),
            new AutoDriveCommand(-initialDistance, driveSubsystem),
            new ArmCommand(Arm.minDegrees, armSubsystem)
        );
    }
    

}
