package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PlacePieceCommand extends SequentialCommandGroup {

    public PlacePieceCommand(Supplier<Height> height, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new ConditionalCommand(
                new PieceOnFloorCommand(armSubsystem, wristSubsystem, gripperSubsystem), 
                new ConeOnPoleCommand(height.get(), armSubsystem, wristSubsystem, gripperSubsystem),
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
