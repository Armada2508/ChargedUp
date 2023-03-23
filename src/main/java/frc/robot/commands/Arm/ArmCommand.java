package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {

    private final double degreesDeadband = 0.5;
    private double velocity;
    private double acceleration;
    private DoubleSupplier targetDegrees;
    private ArmSubsystem armSubsystem;

    /**
     * 
     * @param theta degree to go to, 0 is straight down.
     * @param armSubsystem
     */
    public ArmCommand(double theta, double velocity, double acceleration, ArmSubsystem armSubsystem) {
        this(() -> theta, velocity, acceleration, armSubsystem);
    }

    /**
     * 
     * @param theta degree to go to, 0 is straight down.
     * @param armSubsystem
     */
    public ArmCommand(DoubleSupplier theta, double velocity, double acceleration, ArmSubsystem armSubsystem) {
        this.velocity = velocity;
        this.acceleration = acceleration;
        targetDegrees = theta;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.configMotionMagic(velocity, acceleration);
        armSubsystem.setPosition(targetDegrees.getAsDouble());  
    }

    @Override
    public void execute() {

    }
   
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        double currentDegrees = armSubsystem.getPosition();
        return (currentDegrees < armSubsystem.getTarget()+degreesDeadband && currentDegrees > armSubsystem.getTarget()-degreesDeadband);
    }

    public double getTarget() {
        return targetDegrees.getAsDouble();
    }
    
}
