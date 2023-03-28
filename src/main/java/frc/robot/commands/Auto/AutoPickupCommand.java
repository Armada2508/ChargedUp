package frc.robot.commands.auto;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Gripper;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.ArmWristCommand;
import frc.robot.commands.arm.GripperCommand;
import frc.robot.commands.arm.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;
import frc.robot.subsystems.WristSubsystem;

public class AutoPickupCommand extends SequentialCommandGroup {
    
    public AutoPickupCommand(DriveSubsystem driveSubsystem, PigeonIMU pigeon, TimeOfFlight tof, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem, VisionSubsystem visionSubsystem) {
        addCommands(
            new InstantCommand(() -> visionSubsystem.setPipeline(Target.CONE),visionSubsystem),
            new ArmWristCommand(new ArmCommand(0, 45, 45, armSubsystem), new WristCommand(75, 45, 45, wristSubsystem), -0.5, 10, armSubsystem, wristSubsystem, gripperSubsystem),
            new GripperCommand(Gripper.open, gripperSubsystem, armSubsystem)
            // new WaitUntilCommand(() -> !Double.isNaN(visionSubsystem.getTargetYaw(Target.CONE)))
            // new AutoTurnCommand(visionSubsystem.getTargetYaw(Target.CONE), driveSubsystem, pigeon),
            // new InstantCommand(() -> driveSubsystem.setVelocity(.25, .25)),
            // new GripperTOFCommand(15, tof, gripperSubsystem, armSubsystem)
        );
    }

}
