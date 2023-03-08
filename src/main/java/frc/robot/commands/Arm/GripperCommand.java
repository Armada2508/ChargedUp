package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {

    private final double positionDeadband = 0.005;
    private double velocity;
    private double acceleration;
    private DoubleSupplier position;
    private GripperSubsystem gripperSubsystem;

    public GripperCommand(double position, double velocity, double acceleration, GripperSubsystem gripperSubsystem) {
        this(() -> position, velocity, acceleration, gripperSubsystem);
    }

    public GripperCommand(DoubleSupplier position, double velocity, double acceleration, GripperSubsystem gripperSubsystem) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.gripperSubsystem = gripperSubsystem;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        gripperSubsystem.setPosition(position.getAsDouble());  
    }

    @Override
    public void execute() {
        // System.out.println("Goin Gripper");
    }
   
    @Override
    public void end(boolean interrupted) {
        System.out.println("FINISHED GRIPPER COMMAND");
        gripperSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        double currentPosition = gripperSubsystem.getPosition();
        return (currentPosition < gripperSubsystem.getTarget()+positionDeadband && currentPosition > gripperSubsystem.getTarget()-positionDeadband);
    }

}
