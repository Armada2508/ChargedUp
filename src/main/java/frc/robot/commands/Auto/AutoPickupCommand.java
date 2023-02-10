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
import frc.robot.subsystems.PhotonSubsystem.Orientation;
import frc.robot.subsystems.PhotonSubsystem.Target;
import frc.robot.subsystems.WristSubsystem;

public class AutoPickupCommand extends SequentialCommandGroup {

    private final double armCube = 0.0;
    private final double wristCube = 0.0;
    private final double gripperCube = 0.0;
    private final double armConePortrait = 0.0;
    private final double wristConePortrait = 0.0;
    private final double gripperConePortrait = 0.0;
    private final double armConeLandscape = 0.0;
    private final double wristConeLandscape = 0.0;
    private final double gripperConeLandscape = 0.0;

    private Target lastTarget = Target.CONE;
    private double cubeDistance = 0.0;
    private double coneDistance = 0.0;

    // TODO check wait times
    public AutoPickupCommand(PhotonSubsystem photonSubsystem, DriveSubsystem driveSubsystem, PigeonIMU pigeon, ArmSubsystem ArmSubsystem, WristSubsystem WristSubsystem, GripperSubsystem GripperSubsystem) {
        addCommands(
            new GripperCommand(0, GripperSubsystem),
            new ArmCommand(Arm.minDegrees, ArmSubsystem),
            new SequentialCommandGroup(
                new InstantCommand(() -> photonSubsystem.setPipeline(Vision.cubePipeline), photonSubsystem),
                new WaitCommand(0.1),
                new InstantCommand(() -> cubeDistance = photonSubsystem.getDistanceToTargetInches(Target.CUBE)),
                new InstantCommand(() -> photonSubsystem.setPipeline(Vision.coneLandscapePipeline), photonSubsystem),
                new WaitCommand(0.1),
                new InstantCommand(() -> coneDistance = photonSubsystem.getDistanceToTargetInches(Target.CONE))
            ),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SeekCommand(driveSubsystem, photonSubsystem, pigeon, () -> getTarget(photonSubsystem), 12),

                    new ArmCommand(() -> { /* change values as needed after testing */
                        if (lastTarget == Target.CUBE) return armCube;
                        else if (photonSubsystem.getConeOrientation()== Orientation.LANDSCAPE) return armConeLandscape;
                        else return armConePortrait; //Portrait Orentation
                    }, 
                    ArmSubsystem),

                    new WristCommand(() -> { /* change values as needed after testing */
                        if (lastTarget == Target.CUBE) return wristCube;
                        else if (photonSubsystem.getConeOrientation()== Orientation.LANDSCAPE) return wristConeLandscape;
                        else return wristConePortrait; //Portrait Orentation
                    }, WristSubsystem), 

                    new GripperCommand(() -> { /* change values as needed after testing */ 
                        if (lastTarget == Target.CUBE) return gripperCube;
                        else if (photonSubsystem.getConeOrientation()== Orientation.LANDSCAPE) return gripperConeLandscape;
                        else return gripperConePortrait; //Portrait Orentation
                    }, GripperSubsystem),

                    new ArmCommand(Arm.minDegrees, ArmSubsystem) /* On True for Conditional, Starts at InstantCommand*/
                ),
                new InstantCommand(), /* On False for Conditional */ 
                () -> getTarget(photonSubsystem) != Target.NONE /* Boolean Supplier for Conditional */
            )
        );
    }

    private Target getTarget(PhotonSubsystem photonSubsystem) {
        Target target = Target.CONE;
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
        else { /*both cubeDistance and coneDistance are NaN */
            target = Target.NONE;

        }
        lastTarget = target;
        return target;
    }

    public Target getPreviousTarget() {
        return lastTarget;
    }

}
