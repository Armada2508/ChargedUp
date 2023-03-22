package frc.robot.commands.driving;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;

public class ButterySmoothDriveCommand extends CommandBase {

    private DoubleSupplier joystickSpeed;
    private DoubleSupplier joystickTurn;
    private DoubleSupplier joystickTrim;
    private BooleanSupplier joystickSlow;
    private boolean squareInputs;
    private DriveSubsystem driveSubsystem;
    private SlewRateLimiter limiterNormal = new SlewRateLimiter(Drive.slewRate);

    public ButterySmoothDriveCommand(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DoubleSupplier joystickTrim,  BooleanSupplier joystickSlow, boolean squareInputs, DriveSubsystem driveSubsystem) {
        this.joystickSpeed = joystickSpeed;
        this.joystickTurn = joystickTurn;
        this.joystickTrim = joystickTrim;
        this.joystickSlow = joystickSlow;
        this.squareInputs = squareInputs;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double speed = joystickSpeed.getAsDouble();
        double turn = joystickTurn.getAsDouble();
        double trim = joystickTrim.getAsDouble();

        // Deadband
        speed = processDeadband(speed);
        turn = processDeadband(turn); 
        trim = processDeadband(trim); 
        // Square the inputs
        if (joystickSlow.getAsBoolean()) {
            speed = Drive.slowSpeed * speed;
            turn = Drive.slowSpeed * turn;
            trim = Drive.slowSpeed * trim;
        }
        else if (squareInputs) {
            speed = Math.signum(speed) * (speed * speed);
            turn = Math.signum(turn) * (turn * turn);        
            trim = Math.signum(trim) * (trim * trim);   
        }
        // Slew Rate Limiting and Turn Adjusting
        speed *= Drive.speedAdjustment;
        turn *= Drive.turnAdjustment;
        trim *= Drive.trimAdjustment;
        speed = limiterNormal.calculate(speed);
        // Constant Curvature, WPILib DifferentialDrive#curvatureDriveIK
        turn = turn * speed + trim; 

        double powerFactor = findSpeed((speed - turn), (speed + turn));

        double leftSpeed = (speed - turn) * powerFactor;
        double rightSpeed = (speed + turn) * powerFactor;
        driveSubsystem.setPower(leftSpeed, rightSpeed);
    }

    /**
     * {@link} https://www.chiefdelphi.com/uploads/default/original/3X/b/a/ba7ccfd90bac0934e374dd4459d813cee2903942.pdf
     */
    private double processDeadband(double val) {
        double newVal = val;
        if(Math.abs(val) < Drive.joystickDeadband) {
            newVal = 0;
        }
        else {
            // Point slope form Y = M(X-X0)+Y0.
            newVal = /*M*/ (1 / (1 - Drive.joystickDeadband)) * /*X-X0*/(val + (-Math.signum(val) * Drive.joystickDeadband));
        }
        return newVal;
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
        driveSubsystem.stop();
    }

}
