package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    
    private final int pitchDeadzone = 2; // degrees
    private final int rollDeadzone = 3; // degrees
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
        double roll = 0;
        if (pigeon.getState() == PigeonState.Ready) {
            pitch = pigeon.getPitch();
            roll = pigeon.getRoll();
        }
        double power = -1*pitch/135; // its in reverse for some reason ? ?
        if (roll > rollDeadzone) { 
            subsystem.setPower(0.1, -0.1);
        } 
        else if (roll < -rollDeadzone) {
            subsystem.setPower(-0.1, 0.1);
        } 
        else if (pitch > pitchDeadzone) { 
            subsystem.setPower(power, power);
        } 
        else if (pitch < -pitchDeadzone) {
            subsystem.setPower(power, power);
        } 
        else { // we're in deadzone so not tilted too much so stop moving
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
