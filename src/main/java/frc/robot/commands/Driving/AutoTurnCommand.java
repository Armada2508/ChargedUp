package frc.robot.commands.driving;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurnCommand extends CommandBase {

    private static final PIDController pid = new PIDController(Drive.turnkP, Drive.turnkI, Drive.turnkD);
    static {
        RobotContainer.addPIDToShuffleBoard(pid, "Turning");
    }
    private final DoubleSupplier relativeDegrees;
    private final double deadbandDegrees = 0.5;
    private final double minIntegral = -1;
    private final double maxIntegral = 1;
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
        // System.out.println(pigeon.getYaw() + " " + relativeDegrees.getAsDouble());
        pid.reset();
        pid.setSetpoint(absoluteTarget);
        pid.setTolerance(deadbandDegrees);
        pid.setIntegratorRange(minIntegral, maxIntegral);
        // System.out.println("P: " + pid.getP() + ", I: " + pid.getI() + ", D: " + pid.getD());
    }

    @Override
    public void execute() {
        double speed = pid.calculate(pigeon.getYaw());
        speed = MathUtil.clamp(speed, -Drive.maxTurnSpeed, Drive.maxTurnSpeed);
        double leftSpeed = speed;
        double rightSpeed = -speed;
        driveSubsystem.setPower(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

}
