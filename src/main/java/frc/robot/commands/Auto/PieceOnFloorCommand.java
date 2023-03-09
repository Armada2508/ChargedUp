package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.GripperCommand;
import frc.robot.commands.Arm.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PieceOnFloorCommand extends SequentialCommandGroup {

    public PieceOnFloorCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
           new ArmCommand(-90, 45, 45, armSubsystem),
           new WristCommand(0, 10, 10, wristSubsystem),
           new GripperCommand(0, gripperSubsystem),
           // Reverse
           new ArmCommand(Arm.minDegrees, 45, 45, armSubsystem)
        );   
       
    }

}
