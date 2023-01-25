package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {

    private final int degreesDeadband = 1;
    private double targetDegrees;
    private WristSubsystem subsystem;

    public WristCommand(double theta, WristSubsystem subsystem) {
        targetDegrees = theta;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setPosition(targetDegrees);  
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
        double currentDegrees = subsystem.getPosition();
        return (Math.abs(currentDegrees) < targetDegrees+degreesDeadband);
    }
}
