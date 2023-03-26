package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Gripper;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.GripperCommand;
import frc.robot.commands.arm.GripperTOFCommand;
import frc.robot.commands.arm.WristCommand;
import frc.robot.commands.driving.AutoTurnCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;
import frc.robot.subsystems.WristSubsystem;

public class AltAutoPickupCommand extends SequentialCommandGroup {
    
    public AltAutoPickupCommand(DriveSubsystem driveSubsystem, PigeonIMU pigeon, TimeOfFlight tof, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem, VisionSubsystem visionSubsystem) {
        addCommands(
            new InstantCommand(() -> visionSubsystem.setPipeline(Target.CONE),visionSubsystem),
            new WristCommand(80, 180, 180, wristSubsystem),
            new ArmCommand(20, 45, 45, armSubsystem),
            new GripperCommand(Gripper.open, gripperSubsystem, armSubsystem),
            new WaitUntilCommand(() -> !Double.isNaN(visionSubsystem.distanceFromTargetMeters(Target.CONE))),
            new AutoTurnCommand(visionSubsystem.getTargetYaw(Target.CONE), driveSubsystem, pigeon),
            new InstantCommand(() -> driveSubsystem.setVelocity(.25, .25)),
            new GripperTOFCommand(15, tof, gripperSubsystem, armSubsystem)
        );
    }

}
