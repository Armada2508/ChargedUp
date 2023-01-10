package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CenterCommand extends CommandBase {

    private final double deadbandDegrees = 0.1;
    private VisionSubsystem vision;
    private DriveSubsystem driveSubsystem;

    public CenterCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        vision = visionSubsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double x = vision.getTargetX();
        if (x > deadbandDegrees) { // Target is left
            driveSubsystem.setPower(-.5, .5);
        } else if (x < -deadbandDegrees) {
            driveSubsystem.setPower(.5, -.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setPower(0, 0);
    }
    
    @Override
    public boolean isFinished() {
        double x = vision.getTargetX();
        return (x < deadbandDegrees && x > -deadbandDegrees);
    }

}
