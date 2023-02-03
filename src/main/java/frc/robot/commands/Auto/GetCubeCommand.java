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
    private PigeonIMU pigeon;
    
    private SequentialCommandGroup group;

    public GetCubeCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem, ArmSubsystem ArmSubsystem, WristSubsystem WristSubsystem, GripperSubsystem GripperSubsystem) {
        group = new SequentialCommandGroup(
            new GripperCommand(0, GripperSubsystem),
            new ArmCommand(Arm.minDegrees, ArmSubsystem), 
            new AutoTurnCommand(visionSubsystem::getTargetX, driveSubsystem, pigeon),
            new AutoDriveCommand(() -> visionSubsystem.distanceFromTargetInInches(Target.GAME_PIECE), driveSubsystem),
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

/* 
    leeway between front of robot and cube is 4 inches
    distance from limelight(camera) to the front of the robot is 0 inches
    0 for arm is down
    0 for wrist is straight
  * assume everything works perfectly
*/