package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.commands.Driving.AutoTurnCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class SeekCommand extends CommandBase {

    private SequentialCommandGroup group;

    public SeekCommand(DriveSubsystem driveSubsystem, PhotonSubsystem photonSubsystem, PigeonIMU pigeon) {
        group = new SequentialCommandGroup(
            new AutoTurnCommand(photonSubsystem::getTargetYaw, driveSubsystem, pigeon),
            new WaitCommand(0.1),
            new AutoDriveCommand(() -> photonSubsystem.getDistanceToTargetInches(Target.CONE) - 24, driveSubsystem)
        );
    }

   public Command getCommand() {
        return group;
    }
    
}
