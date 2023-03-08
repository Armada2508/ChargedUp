package frc.robot.commands.Arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class CalibrateArmCommand extends CommandBase {

    private TalonFX talon;
    private ArmSubsystem armSubsystem;
    private GripperSubsystem gripperSubsystem;
    private boolean updateGripper;

    public CalibrateArmCommand(ArmSubsystem armSubsystem, TalonFX talon, GripperSubsystem gripperSubsystem, boolean updateGripper) {
        this.talon = talon;
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        this.updateGripper = updateGripper;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        talon.setNeutralMode(NeutralMode.Brake);
        talon.set(TalonFXControlMode.PercentOutput, -0.04);
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        talon.setNeutralMode(NeutralMode.Coast);
        talon.neutralOutput();
        if (updateGripper) {
            gripperSubsystem.setArmOffset(talon.getSelectedSensorPosition() - armSubsystem.fromAngle(Arm.minDegrees));
        }
        talon.setSelectedSensorPosition(armSubsystem.fromAngle(Arm.minDegrees));
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.pollLimitSwitch();
    }

}
