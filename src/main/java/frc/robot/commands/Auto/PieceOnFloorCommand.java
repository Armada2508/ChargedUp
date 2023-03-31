package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Wrist;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.ArmWristCommand;
import frc.robot.commands.arm.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PieceOnFloorCommand extends SequentialCommandGroup {

    public PieceOnFloorCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new ArmWristCommand(new ArmCommand(10, 100, 75, armSubsystem), new WristCommand(Wrist.maxDegrees, 130, 130, wristSubsystem, armSubsystem), -0.5, 30, armSubsystem, wristSubsystem, gripperSubsystem)
        );   
       
    }

}
