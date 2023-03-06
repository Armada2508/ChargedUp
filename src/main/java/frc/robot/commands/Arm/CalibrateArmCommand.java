package frc.robot.commands.Arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;

public class CalibrateArmCommand extends CommandBase {

    private TalonFX talon;
    private ArmSubsystem armSubsystem;

    public CalibrateArmCommand(ArmSubsystem armSubsystem, TalonFX talon) {
        this.talon = talon;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        talon.set(TalonFXControlMode.PercentOutput, -0.1);
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        talon.neutralOutput();
        talon.setSelectedSensorPosition(armSubsystem.fromAngle(Arm.minDegrees));
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.pollLimitSwitch();
    }

}
