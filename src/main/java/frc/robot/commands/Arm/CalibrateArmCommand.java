package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class CalibrateArmCommand extends CommandBase {

    private TalonFX talonFX;
    private ArmSubsystem armSubsystem;
    private GripperSubsystem gripperSubsystem;

    public CalibrateArmCommand(TalonFX talonFX, ArmSubsystem armSubsystem, GripperSubsystem gripperSubsystem) {
        this.talonFX = talonFX;
        this.armSubsystem = armSubsystem;
        this.gripperSubsystem = gripperSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        talonFX.set(TalonFXControlMode.PercentOutput, -0.08);
    }

    @Override
    public void execute() {
        
    }
   
    @Override
    public void end(boolean interrupted) {
        double calibrateAngle = Arm.minDegrees;
        armSubsystem.stop();
        gripperSubsystem.updateArmOffset(talonFX.getSelectedSensorPosition() - armSubsystem.fromAngle(calibrateAngle));
        talonFX.setSelectedSensorPosition(armSubsystem.fromAngle(calibrateAngle));
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.pollLimitSwitch();
    }

}
