package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurnCommand extends CommandBase {

    private final DoubleSupplier relativeDegrees;
    private final double timeForFinish = 0.1; // seconds
    private double currentTime;
    private PIDController pid = new PIDController(Drive.turnkP, 0, 0);
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
        this(() -> targetDegrees, driveSubsystem, pigeon);
    }

    /**
     * 
     * @param driveSubsystem
     * @param pigeon
     * @param targetDegrees positive is right, negative is left
     */
    public AutoTurnCommand(DoubleSupplier targetDegrees, DriveSubsystem driveSubsystem, PigeonIMU pigeon) {
        this.driveSubsystem = driveSubsystem;
        this.pigeon = pigeon;
        this.relativeDegrees = targetDegrees;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        absoluteTarget = pigeon.getYaw() + relativeDegrees.getAsDouble();
        currentTime = 0;
        pid.reset();
        pid.setSetpoint(absoluteTarget);
        pid.setTolerance(0.1);
    }

    @Override
    public void execute() {
        double speed = pid.calculate(pigeon.getYaw());
        speed = MathUtil.clamp(speed, -0.5, 0.5);
        if (Math.abs(speed) < Drive.minSpeed) speed = Drive.minSpeed * Math.signum(speed);
        driveSubsystem.setPower(speed, -speed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setPower(0, 0);
    }

    @Override
    public boolean isFinished() { 
        if (pid.atSetpoint()) {
            currentTime += 0.02;
        }
        return pid.atSetpoint() && currentTime >= timeForFinish;
    }
}
