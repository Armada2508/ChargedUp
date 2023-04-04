package frc.robot.commands.auto;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Gripper;
import frc.robot.Constants.Wrist;
import frc.robot.commands.arm.GripperCommand;
import frc.robot.commands.arm.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoGripperCommand extends CommandBase {
    
    private boolean seeingCone = false;
    private int detectionFailures = 0;
    private final DriveSubsystem driveSubsystem;
    private final ArmSubsystem armSubsystem;
    private final WristSubsystem wristSubsystem;
    private final GripperSubsystem gripperSubsystem;
    private final TimeOfFlight tof;
    private final double minVelocity = 0.05;
    private final double minDistance = 10; // centimeters
    private final double maxDistance = 30; // centimeters
    
    public AutoGripperCommand(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem, TimeOfFlight tof) {
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.tof = tof;
        addRequirements(gripperSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // System.out.println("Cone: " + seeingCone + " Velocity: " + driveSubsystem.getLeftVelocity() + " TOF: " + tof.getRange()/10 + " DetectionFailures: " + detectionFailures);
        if (gripperSubsystem.isCalibrated()) {
            if (!seeingCone && Math.abs(driveSubsystem.getLeftVelocity()) < minVelocity && Math.abs(driveSubsystem.getRightVelocity()) < minVelocity && tof.isRangeValid() && (tof.getRange()/10) < minDistance) {
                System.out.println("Saw Cone.");
                new SequentialCommandGroup( // gripper close
                    new GripperCommand(Gripper.grabCone, gripperSubsystem, armSubsystem),
                    new WristCommand(Wrist.maxDegrees, 130, 130, wristSubsystem, armSubsystem)
                ).schedule(); 
                seeingCone = true;
            }
            if ((!tof.isRangeValid() || (tof.getRange()/10) > maxDistance)) {
                detectionFailures += 1;
            } else {
                detectionFailures = 0;
            }
            if (detectionFailures > 15) {
                seeingCone = false;
            } 
        }
    }

    @Override
    public void end(boolean interrupted) {
        gripperSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
