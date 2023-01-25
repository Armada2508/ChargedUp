package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {

    private final int degreesDeadband = 1;
    private double targetDegrees;
    private GripperSubsystem subsystem;

    public GripperCommand(double theta, GripperSubsystem subsystem) {
        targetDegrees = theta;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setPercentClosed(targetDegrees);  
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        subsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        double currentDegrees = subsystem.getPercentClosed();
        return (Math.abs(currentDegrees) < targetDegrees+degreesDeadband);
    }

}
