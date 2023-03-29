package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
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
            new ArmWristCommand(new ArmCommand(10, 45, 45, armSubsystem), new WristCommand(Wrist.maxDegrees, 45, 45, wristSubsystem), -0.5, 10, armSubsystem, wristSubsystem, gripperSubsystem),
            new FinishScoreCommand(Units.inchesToMeters(-12), driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem)
        );   
       
    }

}
