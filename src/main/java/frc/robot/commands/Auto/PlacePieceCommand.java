package frc.robot.commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;
import frc.robot.subsystems.WristSubsystem;

public class PlacePieceCommand extends SequentialCommandGroup {

    public PlacePieceCommand(Supplier<Target> target, Supplier<Height> height, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new ConditionalCommand(
                new PieceOnFloorCommand(driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 
                new ConditionalCommand(
                    new ConeOnPoleCommand(height, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem),
                    new CubeOnStationCommand(height, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem),
                    () -> target.get() == Target.CONE
                ), 
                () -> height.get() == Height.BOTTOM
            )
        );
    }

    public enum Height {
        BOTTOM,
        MID,
        HIGH
    }
    
}
