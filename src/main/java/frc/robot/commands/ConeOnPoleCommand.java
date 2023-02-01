package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Wrist;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.GripperCommand;
import frc.robot.commands.Arm.WristCommand;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ConeOnPoleCommand extends CommandBase {

    private SequentialCommandGroup group;

    public ConeOnPoleCommand(Height height, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        group = (height == Height.MID) ? new SequentialCommandGroup(
            new WristCommand(Wrist.maxDegrees, wristSubsystem), // Don't hit pole
            new ArmCommand(90, armSubsystem),
            new AutoDriveCommand(6, driveSubsystem),
            new WristCommand(0, wristSubsystem),
            new GripperCommand(0, gripperSubsystem),
            new WaitCommand(1),
            // Reverse
            new WristCommand(Wrist.maxDegrees, wristSubsystem),
            new AutoDriveCommand(-6, driveSubsystem),
            new ArmCommand(Arm.minDegrees, armSubsystem)
        ) : new SequentialCommandGroup(
            
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
