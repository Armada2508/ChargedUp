package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AprilTagCommand;
import frc.robot.commands.auto.AprilTagCommand.Position;
import frc.robot.commands.auto.PlacePieceCommand;
import frc.robot.commands.auto.PlacePieceCommand.Height;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;
import frc.robot.subsystems.WristSubsystem;

public class PositionAndPlaceCommand extends SequentialCommandGroup {

    private Joystick joystick;
    
    public PositionAndPlaceCommand(Joystick joystick, Supplier<Target> target, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem, VisionSubsystem visionSubsystem, PigeonIMU pigeon) {
        this.joystick = joystick;
        addCommands(
            new AprilTagCommand(this::getPosition, driveSubsystem, visionSubsystem),
            new PlacePieceCommand(target, this::getHeight, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem)
        );
    }

    private Position getPosition() {
        double val = joystick.getRawAxis(0) * -1;
        if (val > 0.5) return Position.RIGHT;
        if (val < -0.5) return Position.LEFT;
        return Position.CENTER;
    }

    private Height getHeight() {
        double val = joystick.getRawAxis(1);
        if (val > 0.5) return Height.HIGH;
        if (val < -0.5) return Height.BOTTOM;
        return Height.MID;
    }

}
