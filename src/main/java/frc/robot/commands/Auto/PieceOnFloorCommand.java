package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Vision;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.GripperCommand;
import frc.robot.commands.Arm.WristCommand;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;
import frc.robot.subsystems.WristSubsystem;

public class PieceOnFloorCommand extends SequentialCommandGroup {

    public PieceOnFloorCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new AutoDriveCommand(() -> {
                return visionSubsystem.distanceFromTargetInInches(Target.HIGH_POLE)-Vision.distanceHighPoleToFrontInches;
            }, driveSubsystem),
           new ArmCommand(0, armSubsystem),
           new WristCommand(0, wristSubsystem),
           new GripperCommand(0, gripperSubsystem),
           // Reverse
           new ArmCommand(Arm.minDegrees, armSubsystem)
        );   
       
    }

}
