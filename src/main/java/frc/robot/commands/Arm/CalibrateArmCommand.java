package frc.robot.commands.Arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.ArmSubsystem;

public class CalibrateArmCommand extends CommandBase {

    private TalonFX armMotor;
    private ArmSubsystem armSubsystem;

    public CalibrateArmCommand(ArmSubsystem armSubsystem, TalonFX talon) {
        System.out.println(talon.getDeviceID());
        this.armMotor = talon;
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armMotor.set(TalonFXControlMode.PercentOutput, -0.1);
    }

    @Override
    public void execute() {
    }
   
    @Override
    public void end(boolean interrupted) {
        armMotor.set(TalonFXControlMode.PercentOutput, 0);
        armMotor.setSelectedSensorPosition(armSubsystem.fromAngle(Arm.minDegrees));
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.pollLimitSwitch();
    }

}
