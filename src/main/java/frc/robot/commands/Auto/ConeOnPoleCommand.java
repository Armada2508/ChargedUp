package frc.robot.commands.Auto;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Wrist;
import frc.robot.InverseKinematics;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.GripperCommand;
import frc.robot.commands.Arm.WristCommand;
import frc.robot.commands.Auto.PlacePieceCommand.Height;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ConeOnPoleCommand extends SequentialCommandGroup {

    private final double initialDistanceMeters = Units.inchesToMeters(6);

    public ConeOnPoleCommand(Supplier<Height> height, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        double x = 0, y = 0;
        if (height.get() == Height.MID) {
            x = 34.75; 
            y = 34; 
        } else {
            x = 43.947;
            y = 14;
        }
        addCommands(
            InverseKinematics.getIKPositionCommand(x, y, armSubsystem, wristSubsystem),
            new AutoDriveCommand(initialDistanceMeters, driveSubsystem),
            new GripperCommand(0, gripperSubsystem),
            new WaitCommand(.5),
            // Reverse
            new WristCommand(Wrist.maxDegrees, 10, 10, wristSubsystem),
            new AutoDriveCommand(-initialDistanceMeters, driveSubsystem),
            new ArmCommand(Arm.minDegrees, 45, 45, armSubsystem)
        );
    }

}
