package frc.robot.commands.Driving;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurnCommand extends CommandBase {

    private final double relativeDegrees;
    private final double kP = 0.008;
    private PIDController pid = new PIDController(kP, 0, 0);
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
        this.relativeDegrees = targetDegrees;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        absoluteTarget = pigeon.getYaw() + relativeDegrees;
        pid.reset();
        pid.setSetpoint(absoluteTarget);
        pid.setTolerance(1);
    }

    @Override
    public void execute() {
        double speed = pid.calculate(pigeon.getYaw());
        speed = MathUtil.clamp(speed, -1, 1);
        if (Math.abs(speed) < 0.1) speed = 0.1 * Math.signum(speed);
        driveSubsystem.setPower(speed, -speed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setPower(0, 0);
    }

    @Override
    public boolean isFinished() { 
        return pid.atSetpoint();
    }
}
