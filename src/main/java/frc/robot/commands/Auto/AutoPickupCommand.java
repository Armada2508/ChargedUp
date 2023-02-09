package frc.robot.commands.Auto;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Vision;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.GripperCommand;
import frc.robot.commands.Arm.WristCommand;
import frc.robot.commands.Driving.SeekCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.PhotonSubsystem.Target;
import frc.robot.subsystems.WristSubsystem;

public class AutoPickupCommand extends SequentialCommandGroup {

    private Target lastTarget = Target.CONE;

    private double cubeDistance;
    private double coneDistance;

    public AutoPickupCommand(PhotonSubsystem photonSubsystem, DriveSubsystem driveSubsystem, PigeonIMU pigeon, ArmSubsystem ArmSubsystem, WristSubsystem WristSubsystem, GripperSubsystem GripperSubsystem) {
        addCommands(
            new GripperCommand(0, GripperSubsystem),
            new ArmCommand(Arm.minDegrees, ArmSubsystem),
            new SequentialCommandGroup(
                new InstantCommand(() -> photonSubsystem.setPipeline(Vision.cubePipeline), photonSubsystem),
                new WaitCommand(0.1),
                new InstantCommand(() -> coneDistance = photonSubsystem.getDistanceToTargetInches(Target.CONE)),
                new InstantCommand(() -> photonSubsystem.setPipeline(Vision.coneLandscapePipeline), photonSubsystem),
                new WaitCommand(0.1),
                new InstantCommand(() -> cubeDistance = photonSubsystem.getDistanceToTargetInches(Target.CONE))
            ),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SeekCommand(driveSubsystem, photonSubsystem, pigeon, () -> getTarget(photonSubsystem), 12),
                    new ArmCommand(20, ArmSubsystem), // theta value is a guess, change as needed
                    new WristCommand(10, WristSubsystem), // theta value is a guess, change as needed
                    new GripperCommand(1, GripperSubsystem),
                    new ArmCommand(Arm.minDegrees, ArmSubsystem) /* On True for Conditional, Starts at InstantCommand*/
                ),
                new InstantCommand(), /* On False for Conditional */ 
                photonSubsystem::hasTargets /* Boolean Supplier for Conditional */
            )
        );
    }

    //~ new WaitCommand(seconds),
    //// switch to cube pipeline, wait 0.1 seconds, grab distance and store in a var
    //// switch to cone pipeline, wait 0.1 seconds, grab distance and store in a var
    // getTarget(coneDistance, cubeDistance)
    // check if the distances are NaN from Double.NaN. 
    // if one is NaN and one isn't, then seek to that one
    // if both are NaN do nothing,
    // if both have distances go to the one closer

    private Target getTarget(double coneDistance, double cubeDistance, PhotonSubsystem photonSubsystem) {
        if ((cubeDistance == Double.NaN) && (coneDistance != Double.NaN)) {
            return Target.CUBE;
        }
        else if ((cubeDistance != Double.NaN) && (coneDistance == Double.NaN)) {
            return Target.CONE;
        }
        else if ((cubeDistance != Double.NaN) && (coneDistance != Double.NaN)) {
            
        }
        else { /*both cubeDistance and coneDistance are NaN */

        }
        lastTarget = target;
        return target;
    }

    public Target getPreviousTarget() {
        return lastTarget;
    }

}
