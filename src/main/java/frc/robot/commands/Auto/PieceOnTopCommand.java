package frc.robot.commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PhotonSubsystem.Target;
import frc.robot.subsystems.WristSubsystem;

public class PieceOnTopCommand extends SequentialCommandGroup {

    PieceOnTopCommand(Supplier<Target> target, Height height, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new ConditionalCommand(
                new ConeOnPoleCommand(height, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem),
                new CubeOnStationCommand(height, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem),
                () -> target.get() == Target.CONE
            )
        );
    }

    public enum Height {
        MID,
        HIGH
    }
    
}
