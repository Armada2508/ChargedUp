package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends CommandBase {

    private final double distanceDeadbandMeters = Units.inchesToMeters(1.1);
    private final DoubleSupplier targetDistance;
    private double absoluteTarget;
    private DriveSubsystem driveSubsystem;

    public AutoDriveCommand(double distanceMeters, DriveSubsystem driveSubsystem) {
        this(() -> distanceMeters, driveSubsystem);
    }
    
    public AutoDriveCommand(DoubleSupplier distanceMeters, DriveSubsystem driveSubsystem) {
        targetDistance = distanceMeters;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        absoluteTarget = driveSubsystem.getleftPostition() + targetDistance.getAsDouble();
        driveSubsystem.configMotionMagic(3, 1);
        driveSubsystem.driveDistance(targetDistance.getAsDouble());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        double currentPos = driveSubsystem.getMotionMagicPosition();
        return (currentPos < absoluteTarget+distanceDeadbandMeters && currentPos > absoluteTarget-distanceDeadbandMeters);
    }

}
