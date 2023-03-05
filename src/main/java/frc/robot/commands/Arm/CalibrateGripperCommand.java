package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Gripper;
import frc.robot.subsystems.GripperSubsystem;

public class CalibrateGripperCommand extends CommandBase {

    private GripperSubsystem gripperSubsystem;

    public CalibrateGripperCommand(GripperSubsystem gripperSubsystem) {
        this.gripperSubsystem = gripperSubsystem;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        gripperSubsystem.setPower(-0.1);
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        gripperSubsystem.setPower(0);
        gripperSubsystem.calibrate(Gripper.min);
        gripperSubsystem.finishedMoving();
    }

    @Override
    public boolean isFinished() {
        return gripperSubsystem.pollLimitSwitch();
    }
    
}
