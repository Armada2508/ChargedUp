package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {

    private final int degreesDeadband = 1;
    private DoubleSupplier targetDegrees;
    private WristSubsystem wristSubsystem;

    /**
     * 
     * @param theta degree to go to, 0 is straight out.
     * @param wristSubsystem
     */
    public WristCommand(double theta, WristSubsystem wristSubsystem) {
       this(() -> theta, wristSubsystem);
    }

    /**
     * 
     * @param theta degree to go to, 0 is straight out.
     * @param wristSubsystem
     */
    public WristCommand(DoubleSupplier theta, WristSubsystem wristSubsystem) {
        targetDegrees = theta;
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        wristSubsystem.setPosition(targetDegrees.getAsDouble());  
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.stop();
        wristSubsystem.holdPosition();
    }

    @Override
    public boolean isFinished() {
        double currentDegrees = wristSubsystem.getPosition();
        return (currentDegrees < targetDegrees.getAsDouble()+degreesDeadband && currentDegrees > targetDegrees.getAsDouble()-degreesDeadband);
    }
}
