package frc.robot.commands.Auto;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.GripperCommand;
import frc.robot.commands.Arm.WristCommand;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.commands.Driving.AutoTurnCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;
import frc.robot.subsystems.WristSubsystem;

public class GetCubeCommand {
    
    private SequentialCommandGroup group;

    public GetCubeCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem, PigeonIMU pigeon, ArmSubsystem ArmSubsystem, WristSubsystem WristSubsystem, GripperSubsystem GripperSubsystem) {
        group = new SequentialCommandGroup(
            new GripperCommand(0, GripperSubsystem),
            new ArmCommand(Arm.minDegrees, ArmSubsystem), 
            new AutoTurnCommand(visionSubsystem::getTargetX, driveSubsystem, pigeon),
            new AutoDriveCommand(() -> visionSubsystem.distanceFromTargetInInches(Target.CONE), driveSubsystem),
            new ArmCommand(20, ArmSubsystem), //theta value is a guess, change as needed
            new WristCommand(10, WristSubsystem), //theta value is a guess, change as needed
            new GripperCommand(1, GripperSubsystem),
            new ArmCommand(Arm.minDegrees, ArmSubsystem)
        );
    }

    public Command getCommand() {
        return group;
    }

}