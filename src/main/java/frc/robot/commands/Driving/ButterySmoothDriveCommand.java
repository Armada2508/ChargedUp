package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;

public class ButterySmoothDriveCommand extends CommandBase {

    private DoubleSupplier joystickSpeed;
    private DoubleSupplier joystickTurn;
    private DoubleSupplier joystickTrim;
    private DriveSubsystem driveSubsystem;
    private SlewRateLimiter limiterNormal = new SlewRateLimiter(Drive.slewRate);
    private final double maxEncoderVelocity = 14000;

    public ButterySmoothDriveCommand(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DoubleSupplier joystickTrim,  DriveSubsystem driveSubsystem) {
        this.joystickSpeed = joystickSpeed;
        this.joystickTurn = joystickTurn;
        this.joystickTrim = joystickTrim;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double speed = joystickSpeed.getAsDouble();
        double turn = joystickTurn.getAsDouble()/Drive.turnAdjustment;
        double trim = joystickTrim.getAsDouble()/Drive.turnAdjustment;
        // Deadband
        if (Math.abs(speed) < Drive.joystickDeadband) speed = 0;
        if (Math.abs(turn) < Drive.joystickDeadband) turn = 0; 
        if (Math.abs(trim) < Drive.joystickDeadband) trim = 0; 

        speed = limiterNormal.calculate(speed);
        // System.out.println(speed + " " + turn + " " + trim);
        turn = turn * Math.abs(speed) + trim; // Constant Curvature
        double powerFactor = findSpeed((speed - turn), (speed + turn));

        double leftSpeed = (speed - turn) * powerFactor;
        double rightSpeed = (speed + turn) * powerFactor;
        // System.out.println(leftSpeed + " " + powerToVelocity(leftSpeed));
        driveSubsystem.setEncoderVelocity(powerToVelocity(leftSpeed), powerToVelocity(rightSpeed));
    }

    private double findSpeed(double left, double right){
        double p = 1;

        if(left > 1){
            p = 1/left;
        } 
        else if(right > 1){
            p = 1/right;
        }
        return p;
    }

    private double powerToVelocity(double power) {
        return power * maxEncoderVelocity;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setPower(0, 0);
    }

}
