package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase{

    private DoubleSupplier joystickSpeed;
    private DoubleSupplier joystickTurn;
    private DriveSubsystem driveSubsystem;
    private SlewRateLimiter limiter = new SlewRateLimiter(Drive.slewRate);

    public DriveCommand(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DoubleSupplier joystickTrim, DriveSubsystem driveSubsystem){
        this.joystickSpeed = joystickSpeed;
        this.joystickTurn = joystickTurn;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double speed = joystickSpeed.getAsDouble();
        double turn = joystickTurn.getAsDouble()/Drive.turnAdjustment;

        // Deadband
        if (Math.abs(speed) < Drive.joystickDeadband) speed = 0;
        if (Math.abs(turn) < Drive.joystickDeadband) turn = 0;

        speed = limiter.calculate(speed);

        double powerFactor = findSpeed((speed - turn), (speed + turn));

        driveSubsystem.setPower(((speed - turn)*powerFactor), ((speed + turn)*powerFactor));
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

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setPower(0, 0);
    }

}
