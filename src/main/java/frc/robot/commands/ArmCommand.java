package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {

    private final int degreesDeadband = 1;
    private double targetDegrees;
    private ArmSubsystem subsystem;

    public ArmCommand(double theta, ArmSubsystem subsystem) {
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
