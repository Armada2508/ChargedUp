package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Gripper;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SafePositionCommand extends ParallelCommandGroup {
    
    /**
     * Creates a parallel command that closes the gripper and moves the wrist up to the limit switch so the mechanism is in a safe position to move the arm.
     * @param wristSubsystem
     * @param gripperSubsystem
     */
    public SafePositionCommand(WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        addCommands(
            new GripperCommand(Gripper.grabCone, gripperSubsystem),
            new WristCommand(Wrist.maxDegrees, 45, 45, wristSubsystem)
        );
    }

}
