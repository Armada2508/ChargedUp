package frc.robot.commands.Driving;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurnCommand extends CommandBase {

    private final int degreesDeadband = 1;
    private final double targetDegrees;
    private double absoluteTarget;
    private DriveSubsystem driveSubsystem;
    private PigeonIMU pigeon;

    /**
     * 
     * @param driveSubsystem
     * @param pigeon
     * @param targetDegrees positive is right, negative is left
     */
    public AutoTurnCommand(double targetDegrees, DriveSubsystem driveSubsystem, PigeonIMU pigeon) {
        this.driveSubsystem = driveSubsystem;
        this.pigeon = pigeon;
        this.targetDegrees = targetDegrees;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        absoluteTarget = pigeon.getYaw() + targetDegrees;
    }

    @Override
    public void execute() {
        double speed = RobotContainer.send.getSelected() * (absoluteTarget - pigeon.getYaw());
        speed = MathUtil.clamp(speed, -1, 1);
        System.out.println(speed);
        if (speed < 0.1 || speed > 0.1) speed = 0.1 * Math.signum(speed);
        driveSubsystem.setPower(speed, -speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("FINISHED TURNING");
        driveSubsystem.setPower(0, 0);
    }

    @Override
    public boolean isFinished() { 
        final double currentDegrees = pigeon.getYaw();
        return (currentDegrees < absoluteTarget+degreesDeadband && currentDegrees > absoluteTarget-degreesDeadband);
    }
}
