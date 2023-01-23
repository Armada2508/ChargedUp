package frc.robot.commands.Driving;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurnCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private PigeonIMU pigeon;
    private double targetDegrees;
    private double startingYaw;

    /**
     * 
     * @param driveSubsystem
     * @param pigeon
     * @param targetDegrees positive is right, negative is left
     */
    public AutoTurnCommand(double targetDegrees, DriveSubsystem driveSubsystem, PigeonIMU pigeon) {
        if (targetDegrees == 0) cancel();
        this.driveSubsystem = driveSubsystem;
        this.pigeon = pigeon;
        // Account for drift, // ! get rid of this on charged up, this was just for testing on everest
        if (targetDegrees > 25) targetDegrees -= 18;
        if (targetDegrees < 25) targetDegrees += 18;
        this.targetDegrees = targetDegrees;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        startingYaw = pigeon.getYaw();
        targetDegrees = targetDegrees + startingYaw;
        if (targetDegrees > 0) {
            driveSubsystem.setPower(0.25, -0.25);
        }
        else if (targetDegrees < 0) {
            driveSubsystem.setPower(-0.25, 0.25);
        }
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
        final int angleRange = 3; // degrees deadband
        final double currentDegrees = pigeon.getYaw();
        return (currentDegrees < targetDegrees + angleRange && currentDegrees > targetDegrees  - angleRange);

    }
}
