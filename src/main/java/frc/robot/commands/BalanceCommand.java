package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    
    private final int pitchDeadzone = 2; // degrees
    private DriveSubsystem subsystem;
    private PigeonIMU pigeon;

    public BalanceCommand(DriveSubsystem subsystem, PigeonIMU pigeon) {
        this.subsystem = subsystem;
        this.pigeon = pigeon;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double pitch = 0;
        if (pigeon.getState() == PigeonState.Ready) {
            pitch = pigeon.getPitch();
        }
        double power = -1*pitch/180; // its in reverse for some reason ? ?
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        if (pitch > pitchDeadzone) { // outside of deadzone start doin shit
            subsystem.setPower(power, power);
        } else if (pitch < -pitchDeadzone) {
            subsystem.setPower(power, power);
        } else { // we're in deadzone so not tilted too much so stop moving
            subsystem.setPower(0, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
