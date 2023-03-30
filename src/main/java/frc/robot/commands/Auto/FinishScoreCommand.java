package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Gripper;
import frc.robot.commands.arm.GripperCommand;
import frc.robot.commands.driving.AutoDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class FinishScoreCommand extends SequentialCommandGroup {
    
    public FinishScoreCommand(double backupMeters, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new GripperCommand(Gripper.open, gripperSubsystem, armSubsystem),
            new AutoDriveCommand(-backupMeters, 1.5, 0.5, driveSubsystem),
            new StoreCommand(armSubsystem, wristSubsystem, gripperSubsystem)
        );
    }

}
