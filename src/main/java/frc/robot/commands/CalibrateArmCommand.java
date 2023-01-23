package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class CalibrateArmCommand extends CommandBase {

    private ArmSubsystem subsystem;

    public CalibrateArmCommand(ArmSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setPower(-0.1);
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        subsystem.setPower(0);
        subsystem.calibrate();
    }

    @Override
    public boolean isFinished() {
        return subsystem.pollLimitSwitch();
    }
    
}
