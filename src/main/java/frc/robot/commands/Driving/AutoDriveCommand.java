package frc.robot.commands.driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends CommandBase {

    private final double distanceDeadbandMeters = Units.inchesToMeters(0.05);
    private final DoubleSupplier targetDistance;
    private double velocity;
    private double acceleration;
    private DriveSubsystem driveSubsystem;

    public AutoDriveCommand(double distanceMeters, double velocity, double acceleration, DriveSubsystem driveSubsystem) {
        this(() -> distanceMeters, velocity, acceleration, driveSubsystem);
    }
    
    public AutoDriveCommand(DoubleSupplier distanceMeters, double velocity, double acceleration, DriveSubsystem driveSubsystem) {
        targetDistance = distanceMeters;
        this.driveSubsystem = driveSubsystem;
        this.velocity = velocity;
        this.acceleration = acceleration;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.configMotionMagic(velocity, acceleration);
        driveSubsystem.driveDistance(targetDistance.getAsDouble());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        double currentPos = driveSubsystem.getLeftPostition();
        return (currentPos < driveSubsystem.getTarget()+distanceDeadbandMeters && currentPos > driveSubsystem.getTarget()-distanceDeadbandMeters);
    }

}
