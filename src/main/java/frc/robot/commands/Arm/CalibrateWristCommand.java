package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class CalibrateWristCommand extends CommandBase {
    
    private WristSubsystem wristSubsystem;
    private GripperSubsystem gripperSubsystem;

    public CalibrateWristCommand(WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        this.wristSubsystem = wristSubsystem;
        this.gripperSubsystem = gripperSubsystem;
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
        double calibrateAngle = 0;
        wristSubsystem.stop();
        gripperSubsystem.setWristOffset(wristSubsystem.getSensorPosition() - wristSubsystem.fromAngle(calibrateAngle));
        wristSubsystem.calibrate(wristSubsystem.fromAngle(calibrateAngle));
    }

    @Override
    public boolean isFinished() {
        return wristSubsystem.pollLimitSwitch();
    }

}
