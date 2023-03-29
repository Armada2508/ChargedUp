package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Gripper;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {

    private final double positionDeadband = 0.01;
    private DoubleSupplier position;
    private GripperSubsystem gripperSubsystem;
    private ArmSubsystem armSubsystem;

    public GripperCommand(double position, GripperSubsystem gripperSubsystem, ArmSubsystem armSubsystem) {
        this(() -> position, gripperSubsystem, armSubsystem);
    }

    public GripperCommand(DoubleSupplier position, GripperSubsystem gripperSubsystem, ArmSubsystem armSubsystem) {
        this.position = position;
        this.gripperSubsystem = gripperSubsystem;
        this.armSubsystem = armSubsystem;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        double pos = position.getAsDouble();
        if (armSubsystem.insideFrame() && pos < Gripper.closed) {
            cancel();
            return;
        } else {
            gripperSubsystem.setPosition(pos);  
        }
    }

    @Override
    public void execute() {
        
    }
   
    @Override
    public void end(boolean interrupted) {
        gripperSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        double currentPosition = gripperSubsystem.getPhysicalPosition();
        double target = gripperSubsystem.getPhysicalTarget();
        return (Math.abs(currentPosition - target) < positionDeadband) || (gripperSubsystem.pollLimitSwitch());
    }

}
