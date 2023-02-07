package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends CommandBase {

    private final double distanceDeadband = 1;
    private final DoubleSupplier targetDistance;
    private double absoluteTarget;
    private DriveSubsystem driveSubsystem;

    public AutoDriveCommand(double distanceInches, DriveSubsystem driveSubsystem) {
        this(() -> distanceInches, driveSubsystem);
    }

    public AutoDriveCommand(DoubleSupplier distanceInches, DriveSubsystem driveSubsystem) {
        targetDistance = distanceInches;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        absoluteTarget = driveSubsystem.getRightPostition() + targetDistance.getAsDouble();
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
        double currentPos = driveSubsystem.getRightPostition();
        return (currentPos < absoluteTarget+distanceDeadband && currentPos > absoluteTarget-distanceDeadband);
    }


}
