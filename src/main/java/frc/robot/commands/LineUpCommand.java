package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

/**
 * This will center the robot 
 */
public class LineUpCommand extends CommandBase {

    private final double deadbandDegrees = 3;
    private final double deadbandDistance = 3;
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
        if (target == Target.CONE) {
            desiredDistance = 6;
            vision.limelightOFF();
            vision.setPipeline(Vision.colorPipeline);
        } else {
            desiredDistance = 40;
            vision.limelightON();
            vision.setPipeline(Vision.reflectionPipeline);
        }
        if (!vision.hasTarget()) cancel();
    }

    @Override
    public void execute() {
        double x = vision.getTargetX();
        double currentDistance = vision.distanceFromTargetInInches(target);
        double leftSpeed = 0;
        double rightSpeed = 0;
        // System.out.println("Angle: " + x);
        // System.out.println("Vertical: " + vision.getTargetY());
        // System.out.println("Current Distance: " + currentDistance);
        final double limelightMaxDegrees = 29.8;
        double proportionalAngle = Math.abs(x) / (limelightMaxDegrees/4); // PID but just P
        if (proportionalAngle > 1) proportionalAngle = 1;
        if (x > deadbandDegrees) { // Target is right
            leftSpeed += 0.1 * proportionalAngle;
            rightSpeed -= 0.1 * proportionalAngle;
        } else if (x < -deadbandDegrees) { // Target is left
            leftSpeed -= 0.1 * proportionalAngle;
            rightSpeed += 0.1 * proportionalAngle;
        } else if (currentDistance > desiredDistance+deadbandDistance) { // Too far from target
            leftSpeed += 0.125;
            rightSpeed += 0.125;
        } else if (currentDistance < desiredDistance-deadbandDistance) { // Too close to target
            leftSpeed -= 0.125;
            rightSpeed -= 0.125;
        } 
        driveSubsystem.setPower(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setPower(0, 0);
    }
    
    @Override
    public boolean isFinished() {
        double x = vision.getTargetX();
        double currentDistance = vision.distanceFromTargetInInches(target);
        return (
            Math.abs(x) < deadbandDegrees 
            && Math.abs(currentDistance) < desiredDistance+deadbandDistance
        );
    }

}
