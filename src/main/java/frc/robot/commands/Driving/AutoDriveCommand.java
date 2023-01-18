package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveCommand extends CommandBase {
   
    private double targetDistance;
    private DriveSubsystem subsystem;

    public AutoDriveCommand(double distanceInches, DriveSubsystem subsystem) {
        targetDistance = distanceInches;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
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
        double deadband = 0.1;
        double currentPos = subsystem.getRightPostition();
        System.out.println(currentPos + " " + targetDistance);
        return (currentPos > targetDistance-deadband && currentPos < targetDistance+deadband);
    }


}
