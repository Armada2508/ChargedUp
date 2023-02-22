package frc.robot.commands.Auto;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.InverseKinematics;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.GripperCommand;
import frc.robot.commands.Driving.SeekCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Orientation;
import frc.robot.subsystems.VisionSubsystem.Pipeline;
import frc.robot.subsystems.WristSubsystem;

public class AutoPickupCommand extends SequentialCommandGroup {

    private final double distanceFromTargetInches = 12;
    private Pipeline lastTarget = Pipeline.CONE;

    public AutoPickupCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem, PigeonIMU pigeon, ArmSubsystem ArmSubsystem, WristSubsystem WristSubsystem, GripperSubsystem GripperSubsystem) {
        addCommands(
            new GripperCommand(0, GripperSubsystem),
            new ArmCommand(Arm.minDegrees, ArmSubsystem),
            new InstantCommand(() -> visionSubsystem.setPipeline(Pipeline.CONE),visionSubsystem),
            new WaitCommand(0.05),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SeekCommand(driveSubsystem, visionSubsystem, pigeon, this::getPreviousTarget, distanceFromTargetInches),
                    new ConditionalCommand(
                        new ConditionalCommand(
                            InverseKinematics.getIKPositionCommand(12.0, 0.0, ArmSubsystem, WristSubsystem), /*On True  | Landscape Orientation*/
                            InverseKinematics.getIKPositionCommand(12.0, 0.0, ArmSubsystem, WristSubsystem), /*On False | Portrait Orientation*/
                            () -> visionSubsystem.getTargetOrientation(Pipeline.CONE) == Orientation.LANDSCAPE  /*Boolean Supplier*/
                            ), /*On True  | Target.Cone*/
                        InverseKinematics.getIKPositionCommand(12.0, 0.0, ArmSubsystem, WristSubsystem), /*On False | Target.Cube*/
                        () -> getPreviousTarget() == Pipeline.CONE  /*Boolean Supplier*/
                    ),
                    new ArmCommand(Arm.minDegrees, ArmSubsystem) /* On True for Conditional, Starts at InstantCommand*/
                ),
                new InstantCommand(), /* On False for Conditional */ 
                () -> getTarget(visionSubsystem) != Pipeline.NONE /* Boolean Supplier for Conditional */
            )
        );
    }

    private Pipeline getTarget(VisionSubsystem visionSubsystem) {
        Pipeline target = Pipeline.CONE;
        double coneDistance = visionSubsystem.distanceFromTargetInInches(Pipeline.CONE); 
        double cubeDistance = visionSubsystem.distanceFromTargetInInches(Pipeline.CUBE);
        if ((cubeDistance == Double.NaN) && (coneDistance != Double.NaN)) {
            target = Pipeline.CONE;
        }
        else if ((cubeDistance != Double.NaN) && (coneDistance == Double.NaN)) {
            target = Pipeline.CUBE;
        }
        else if ((cubeDistance != Double.NaN) && (coneDistance != Double.NaN)) {
            if (cubeDistance > coneDistance) {
                target = Pipeline.CUBE;
            }
            target = Pipeline.CONE;
        }
        else { 
            target = Pipeline.NONE;
        }
        lastTarget = target;
        return target;
    }

    public Pipeline getPreviousTarget() {
        return lastTarget;
    }

}
