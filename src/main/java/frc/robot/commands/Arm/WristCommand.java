package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {

    private final int degreesDeadband = 1;
    private double targetDegrees;
    private WristSubsystem wristSubsystem;

    public WristCommand(double theta, WristSubsystem wristSubsystem) {
        targetDegrees = theta;
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        wristSubsystem.setPosition(targetDegrees);  
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        double currentDegrees = wristSubsystem.getPosition();
        return (Math.abs(currentDegrees) < targetDegrees+degreesDeadband);
    }
}
