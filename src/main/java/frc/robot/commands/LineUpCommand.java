package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Pole;
import frc.robot.subsystems.VisionSubsystem.Target;

/**
 * This will center the robot 
 */
public class LineUpCommand extends CommandBase {

    private final double deadbandDegrees = 1.5;
    private final double deadbandDistance = 1;
    private final Target target;
    private double desiredDistance; // inches
    private VisionSubsystem vision;
    private DriveSubsystem driveSubsystem;

    public LineUpCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Target target) {
        vision = visionSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.target = target;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        if (target == Target.POLE) {
            desiredDistance = 40;
            vision.limelightON();
            vision.setPipeline(Vision.reflectionPipeline);
        } else {
            desiredDistance = 6;
            vision.limelightOFF();
            vision.setPipeline(Vision.colorPipeline);
        }
        if (!vision.hasTarget()) cancel();
    }

    @Override
    public void execute() {
        double x = vision.getTargetX();
        double speed = 0.1;
        double currentDistance = vision.distanceFromTargetInInches(Pole.MID_POLE);
        currentDistance = desiredDistance;
        if (x < deadbandDegrees) { // Target is left
            driveSubsystem.setPower(-speed, speed);
        } else if (x > -deadbandDegrees) { // Target is right
            driveSubsystem.setPower(speed, -speed);
        } 
        // else if (currentDistance > desiredDistance+deadbandDistance) {
        //     driveSubsystem.setPower(speed, speed);
        // } else if (currentDistance < desiredDistance-deadbandDistance) {
        //     driveSubsystem.setPower(-speed, -speed);
        // }
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
            Math.abs(x) < deadbandDegrees 
            && Math.abs(currentDistance) < desiredDistance+deadbandDistance
        );
    }

}
