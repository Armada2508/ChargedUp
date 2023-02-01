package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class CalibrateArmCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public CalibrateArmCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setPower(-0.1);
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setPower(0);
        armSubsystem.calibrate();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.pollLimitSwitch();
    }
    
}
