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
    
    private static final PIDController rollController = new PIDController(Balance.rollkP, Balance.rollkI, Balance.rollkD);
    static {
        RobotContainer.addPIDToShuffleBoard(rollController, "Balance Roll");
    }
    private DriveSubsystem driveSubsystem;
    private PigeonIMU pigeon;

    public BalanceCommand(DriveSubsystem driveSubsystem, PigeonIMU pigeon) {
        this.driveSubsystem = driveSubsystem;
        this.pigeon = pigeon;
        addRequirements(driveSubsystem);
        rollController.setSetpoint(0);
    }

    @Override
    public void initialize() {
        rollController.reset();
    }

    @Override
    public void execute() {
        double currentPitch = 0;
        double currentRoll = 0;
        if (pigeon.getState() == PigeonState.Ready) {
            currentPitch = pigeon.getPitch() + Balance.pitchOffset;
            currentRoll = pigeon.getRoll() + Balance.rollOffset;
        }
        if (Math.abs(currentPitch) < Balance.balanceAngle) {
            driveSubsystem.stop();
        } else {
            double pitchSpeed = Balance.pitchSpeed * -Math.signum(currentPitch);
            double rollSpeed = rollController.calculate(currentRoll);
            if (currentPitch > 0) rollSpeed *= -1; // Roll is opposite in the other direction for some reason.
            // Clamp maximum
            double leftSpeed = MathUtil.clamp(pitchSpeed+rollSpeed, -Balance.maxSpeed, Balance.maxSpeed);
            double rightSpeed = MathUtil.clamp(pitchSpeed-rollSpeed, -Balance.maxSpeed, Balance.maxSpeed);
            // System.out.println("LeftSpeed: " + leftSpeed + " RightSpeed: " + rightSpeed);
            driveSubsystem.setPower(leftSpeed, rightSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
