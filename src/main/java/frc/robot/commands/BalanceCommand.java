package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Balance;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Balances the robot on the charge substation.
 */
public class BalanceCommand extends CommandBase {
    
    private boolean onChargeStation = false;
    private final boolean reverse;
    private final DriveSubsystem driveSubsystem;
    private final PigeonIMU pigeon;

    public BalanceCommand(boolean reverse, DriveSubsystem driveSubsystem, PigeonIMU pigeon) {
        this.reverse = reverse;
        this.driveSubsystem = driveSubsystem;
        this.pigeon = pigeon;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        onChargeStation = false;
    }

    @Override
    public void execute() {
        double currentPitch = pigeon.getPitch() + Balance.pitchOffset;
        if (Math.abs(currentPitch) > Balance.stationAngle) {
            onChargeStation = true;
        }
        double pitchSpeed = reverse ? -Balance.pitchSpeed : Balance.pitchSpeed;
        driveSubsystem.setPower(pitchSpeed, pitchSpeed);
        System.out.println("Pitch: " + currentPitch + " On Charge Station: " + onChargeStation);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Stopped Balancing.");
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(pigeon.getPitch()) < Balance.angleToStop && onChargeStation);

    }

}
