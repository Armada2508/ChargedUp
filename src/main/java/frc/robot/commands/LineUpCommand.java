package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Pole;

/**
 * This will center the robot 
 */
public class LineUpCommand extends CommandBase {

    private final double deadbandDegrees = 1;
    private final double deadbandDistance = 1;
    private final double desiredDistance = 40; // inches
    private VisionSubsystem vision;
    private DriveSubsystem driveSubsystem;

    public LineUpCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        vision = visionSubsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        if (!vision.hasTarget()) cancel();
    }

    @Override
    public void execute() {
        double x = vision.getTargetX();
        double speed = 0.25;
        double currentDistance = vision.distanceFromTargetInInches(Pole.MID_POLE);
        currentDistance = desiredDistance;
        if (x > deadbandDegrees) { // Target is left
            driveSubsystem.setPower(-speed, speed);
        } else if (x < -deadbandDegrees) { // Target is right
            driveSubsystem.setPower(speed, -speed);
        } else if (currentDistance > desiredDistance+deadbandDistance) {
            driveSubsystem.setPower(speed, speed);
        } else if (currentDistance < desiredDistance-deadbandDistance) {
            driveSubsystem.setPower(-speed, -speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setPower(0, 0);
    }
    
    @Override
    public boolean isFinished() {
        double x = vision.getTargetX();
        double currentDistance = vision.distanceFromTargetInInches(Pole.MID_POLE);
        currentDistance = desiredDistance;
        return (
            x < deadbandDegrees 
            && x > -deadbandDegrees 
            && currentDistance > desiredDistance-deadbandDistance
            && currentDistance < desiredDistance+deadbandDistance
        );
    }

}
