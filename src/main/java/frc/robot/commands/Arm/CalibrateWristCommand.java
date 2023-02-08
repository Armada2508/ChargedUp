package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class CalibrateWristCommand extends CommandBase {
    
    private WristSubsystem wristSubsystem;

    public CalibrateWristCommand(WristSubsystem wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        wristSubsystem.setPower(-0.1);
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.setPower(0);
        wristSubsystem.calibrate();
    }

    @Override
    public boolean isFinished() {
        return wristSubsystem.pollLimitSwitch();
    }

}
