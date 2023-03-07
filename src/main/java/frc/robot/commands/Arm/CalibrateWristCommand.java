package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Wrist;
import frc.robot.subsystems.WristSubsystem;

public class CalibrateWristCommand extends CommandBase {
    
    private WristSubsystem wristSubsystem;

    public CalibrateWristCommand(WristSubsystem wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        wristSubsystem.setPower(-0.05);
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.stop();
        wristSubsystem.calibrate(wristSubsystem.fromAngle(Wrist.minDegrees));
    }

    @Override
    public boolean isFinished() {
        return wristSubsystem.pollLimitSwitch();
    }

}
