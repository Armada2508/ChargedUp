package frc.robot.commands.Auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Wrist;
import frc.robot.InverseKinematics;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.GripperCommand;
import frc.robot.commands.Arm.WristCommand;
import frc.robot.commands.Auto.PieceOnTopCommand.Height;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class CubeOnStationCommand extends SequentialCommandGroup {

    private final double initialDistanceMeters = Units.inchesToMeters(6);

    public CubeOnStationCommand(Height height, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        double x = 0, y = 0;
        if (height == Height.MID) {
            x = 14.25; 
            y = 23.5; 
        } else {
            x = 31.625; 
            y = 35.5; 
        }
        addCommands(
            InverseKinematics.getIKPositionCommand(x, y, armSubsystem, wristSubsystem),
            new AutoDriveCommand(initialDistanceMeters, driveSubsystem),
            new GripperCommand(0, gripperSubsystem),
            new WaitCommand(.5),
            // Reverse
            new WristCommand(Wrist.maxDegrees, wristSubsystem),
            new AutoDriveCommand(-initialDistanceMeters, driveSubsystem),
            new ArmCommand(Arm.minDegrees, armSubsystem)
        );
    }

}
