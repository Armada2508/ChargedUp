package frc.robot.commands.Auto;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.InverseKinematics;
import frc.robot.Lib.util.BetterPair;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.GripperCommand;
import frc.robot.commands.Driving.SeekCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Orientation;
import frc.robot.subsystems.VisionSubsystem.Target;
import frc.robot.subsystems.WristSubsystem;

public class AutoPickupCommand extends SequentialCommandGroup {

    private final double distanceFromTargetMeters = 12;
    private final BetterPair<Double, Double> coneUp = new BetterPair<>(12.0, 0.0);
    private final BetterPair<Double, Double> coneDown = new BetterPair<>(12.0, 0.0);
    private final BetterPair<Double, Double> cube = new BetterPair<>(12.0, 0.0);
    private Target lastTarget = Target.CONE;

    public AutoPickupCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem, PigeonIMU pigeon, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new GripperCommand(0, gripperSubsystem),
            new ArmCommand(Arm.minDegrees, 45, 45, armSubsystem),
            new InstantCommand(() -> visionSubsystem.setPipeline(Target.CONE),visionSubsystem),
            new WaitCommand(0.05),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SeekCommand(this::getPreviousTarget, distanceFromTargetMeters, driveSubsystem, visionSubsystem, pigeon),
                    new ConditionalCommand(
                        new ConditionalCommand(
                            InverseKinematics.getIKPositionCommand(coneDown.getFirst(), coneDown.getSecond(), armSubsystem, wristSubsystem), /*On True  | Landscape Orientation*/
                            InverseKinematics.getIKPositionCommand(coneUp.getFirst(), coneUp.getSecond(), armSubsystem, wristSubsystem), /*On False | Portrait Orientation*/
                            () -> visionSubsystem.getTargetOrientation(Target.CONE) == Orientation.LANDSCAPE  /*Boolean Supplier*/
                            ), /*On True  | Target.Cone*/
                        InverseKinematics.getIKPositionCommand(cube.getFirst(), cube.getSecond(), armSubsystem, wristSubsystem), /*On False | Target.Cube*/
                        () -> getPreviousTarget() == Target.CONE  /*Boolean Supplier*/
                    ),
                    new ArmCommand(Arm.minDegrees, 45, 45, armSubsystem) /* On True for Conditional, Starts at InstantCommand*/
                ),
                new InstantCommand(), /* On False for Conditional */ 
                () -> getTarget(visionSubsystem) != Target.NONE /* Boolean Supplier for Conditional */
            )
        );
    }

    private Target getTarget(VisionSubsystem visionSubsystem) {
        Target target = Target.CONE;
        double coneDistance = visionSubsystem.distanceFromTargetMeters(Target.CONE); 
        double cubeDistance = visionSubsystem.distanceFromTargetMeters(Target.CUBE);
        if ((cubeDistance == Double.NaN) && (coneDistance != Double.NaN)) {
            target = Target.CONE;
        }
        else if ((cubeDistance != Double.NaN) && (coneDistance == Double.NaN)) {
            target = Target.CUBE;
        }
        else if ((cubeDistance != Double.NaN) && (coneDistance != Double.NaN)) {
            if (cubeDistance > coneDistance) {
                target = Target.CUBE;
            }
            target = Target.CONE;
        }
        else { 
            target = Target.NONE;
        }
        lastTarget = target;
        return target;
    }

    public Target getPreviousTarget() {
        return lastTarget;
    }

}
