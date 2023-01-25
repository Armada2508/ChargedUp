package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends CommandBase {

    private final double distanceDeadband = 0.1;
    private final double targetDistance;
    private double absoluteTarget;
    private DriveSubsystem subsystem;

    public AutoDriveCommand(double distanceInches, DriveSubsystem subsystem) {
        targetDistance = distanceInches;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        absoluteTarget = subsystem.getRightPostition() + targetDistance;
        subsystem.driveDistance(targetDistance);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        double currentPos = subsystem.getRightPostition();
        return (currentPos < absoluteTarget+distanceDeadband && currentPos > absoluteTarget-distanceDeadband);
    }


}
