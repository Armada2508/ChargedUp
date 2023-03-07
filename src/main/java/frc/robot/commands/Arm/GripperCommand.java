package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {

    private final double percentDeadband = 0.05;
    private double velocity;
    private double acceleration;
    private DoubleSupplier percentClosed;
    private GripperSubsystem gripperSubsystem;

    public GripperCommand(double percentClosed, GripperSubsystem gripperSubsystem) {
       this(() -> percentClosed, 0.1, 0.1, gripperSubsystem);
    }

    public GripperCommand(DoubleSupplier percentClosed, double velocity, double acceleration, GripperSubsystem gripperSubsystem) {
        this.percentClosed = percentClosed;
        this.gripperSubsystem = gripperSubsystem;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        gripperSubsystem.configMotionMagic(velocity, acceleration);
        gripperSubsystem.setPercentClosed(percentClosed.getAsDouble());  
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        gripperSubsystem.stop();
        gripperSubsystem.finishedMoving();
    }

    @Override
    public boolean isFinished() {
        double currentPercent = gripperSubsystem.getPercentClosed();
        return (currentPercent < gripperSubsystem.getTarget()+percentDeadband && currentPercent > gripperSubsystem.getTarget()-percentDeadband);
    }

}
