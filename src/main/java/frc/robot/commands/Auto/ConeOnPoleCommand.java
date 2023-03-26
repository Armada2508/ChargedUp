package frc.robot.commands.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Wrist;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.GripperCommand;
import frc.robot.commands.arm.WristCommand;
import frc.robot.commands.auto.PlacePieceCommand.Height;
import frc.robot.commands.driving.AutoDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ConeOnPoleCommand extends SequentialCommandGroup {

    private final double initialDistanceMeters = Units.inchesToMeters(6);

    public ConeOnPoleCommand(Supplier<Height> height, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        double arm = 0, wrist = Wrist.maxDegrees;
        if (height.get() == Height.MID) {
            arm = 0; 
            wrist = Wrist.maxDegrees; 
        } else {
            arm = 0;
            wrist = Wrist.maxDegrees;
        }
        addCommands(
            new ArmCommand(arm, 45, 45, armSubsystem),
            new WristCommand(wrist, 45, 45, wristSubsystem),
            new AutoDriveCommand(initialDistanceMeters, 1, 1, driveSubsystem),
            new GripperCommand(0, gripperSubsystem),
            new WaitCommand(.5),
            // Reverse
            new WristCommand(Wrist.maxDegrees, 10, 10, wristSubsystem),
            new AutoDriveCommand(-initialDistanceMeters, 1, 1, driveSubsystem),
            new ArmCommand(Arm.minDegrees, 45, 45, armSubsystem)
        );
    }

}
