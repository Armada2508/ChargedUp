package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {

    private final int degreesDeadband = 1;
    private DoubleSupplier percentClosed;
    private GripperSubsystem gripperSubsystem;

    public GripperCommand(double percentClosed, GripperSubsystem gripperSubsystem) {
       this(() -> percentClosed, gripperSubsystem);
    }

    public GripperCommand(DoubleSupplier percentClosed, GripperSubsystem gripperSubsystem) {
        this.percentClosed = percentClosed;
        this.gripperSubsystem = gripperSubsystem;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        gripperSubsystem.setPercentClosed(percentClosed.getAsDouble());  
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        gripperSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        double currentDegrees = gripperSubsystem.getPercentClosed();
        return (Math.abs(currentDegrees) < percentClosed.getAsDouble()+degreesDeadband);
    }

}
