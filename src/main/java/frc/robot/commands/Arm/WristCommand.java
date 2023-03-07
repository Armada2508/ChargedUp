package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {

    private final double degreesDeadband = 0.5;
    private double velocity;
    private double acceleration;
    private DoubleSupplier targetDegrees;
    private WristSubsystem wristSubsystem;

    /**
     * 
     * @param theta degree to go to, 0 is straight out.
     * @param wristSubsystem
     */
    public WristCommand(double theta, WristSubsystem wristSubsystem) {
       this(() -> theta, 30, 30, wristSubsystem);
    }

    /**
     * 
     * @param theta degree to go to, 0 is straight out.
     * @param wristSubsystem
     */
    public WristCommand(DoubleSupplier theta, double velocity, double acceleration, WristSubsystem wristSubsystem) {
        this.velocity = velocity;
        this.acceleration = acceleration;
        targetDegrees = theta;
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        wristSubsystem.configMotionMagic(velocity, acceleration);
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
        return (currentDegrees < wristSubsystem.getTarget()+degreesDeadband && currentDegrees > wristSubsystem.getTarget()-degreesDeadband);
    }
}
