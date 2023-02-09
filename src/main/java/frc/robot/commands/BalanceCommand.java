package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Balance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Balances the robot on the charge substation and when it is balanced, turn perpendicular.
 */
public class BalanceCommand extends CommandBase {
    
    private static final PIDController pitchController = new PIDController(Balance.pitchkP, Balance.pitchkI, Balance.pitchkD);
    private static final PIDController rollController = new PIDController(Balance.rollkP, Balance.rollkI, Balance.rollkD);
    static {
        RobotContainer.addPIDToShuffleBoard(pitchController, "Balance Pitch");
        RobotContainer.addPIDToShuffleBoard(rollController, "Balance Roll");
    }
    private DriveSubsystem driveSubsystem;
    private PigeonIMU pigeon;

    public BalanceCommand(DriveSubsystem driveSubsystem, PigeonIMU pigeon) {
        this.driveSubsystem = driveSubsystem;
        this.pigeon = pigeon;
        addRequirements(driveSubsystem);
        pitchController.setSetpoint(0);
        rollController.setSetpoint(0);
    }

    @Override
    public void initialize() {
        pitchController.reset();
        rollController.reset();
        driveSubsystem.brake();
    }

    @Override
    public void execute() {
        double currentPitch = 0;
        double currentRoll = 0;
        if (pigeon.getState() == PigeonState.Ready) {
            currentPitch = pigeon.getPitch() + Balance.pitchOffset;
            currentRoll = pigeon.getRoll() + Balance.rollOffset;
            // System.out.println("Pitch: " + currentPitch + " Roll: " + currentRoll);
        }
        double pitchSpeed = pitchController.calculate(currentPitch);
        double rollSpeed = rollController.calculate(currentRoll);
        if (currentPitch > 0) rollSpeed *= -1; // When going the other way its opposite it's flipped.
        // Clamp maximum
        double leftSpeed = MathUtil.clamp(pitchSpeed+rollSpeed, -Balance.maxSpeed, Balance.maxSpeed);
        double rightSpeed = MathUtil.clamp(pitchSpeed-rollSpeed, -Balance.maxSpeed, Balance.maxSpeed);
        // System.out.println("LeftSpeed: " + leftSpeed + " RightSpeed: " + rightSpeed);
        driveSubsystem.setPower(leftSpeed, rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
