package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class PlacePieceCommand extends SequentialCommandGroup {

    private static Height lastHeight;

    public PlacePieceCommand(Supplier<Height> height, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new InstantCommand(() -> lastHeight = height.get()),
            new ConditionalCommand(
                new PieceOnFloorCommand(armSubsystem, wristSubsystem, gripperSubsystem), 
                new ConeOnPoleCommand(lastHeight, armSubsystem, wristSubsystem, gripperSubsystem),
                () -> lastHeight == Height.BOTTOM
            )
        );
    }

    public static Height getLastHeight() {
        return lastHeight;
    }

    public enum Height {
        BOTTOM,
        MID,
        HIGH
    }
    
}
