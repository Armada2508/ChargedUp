package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCommand extends CommandBase {

    private final int degreesDeadband = 1;
    private double percentClosed;
    private GripperSubsystem gripperSubsystem;

    public GripperCommand(double percentClosed, GripperSubsystem gripperSubsystem) {
        this.percentClosed = percentClosed;
        this.gripperSubsystem = gripperSubsystem;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
        gripperSubsystem.setPercentClosed(percentClosed);  
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
        return (Math.abs(currentDegrees) < percentClosed+degreesDeadband);
    }

}
