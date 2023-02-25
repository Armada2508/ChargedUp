package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Gripper;

public class GripperSubsystem extends SubsystemBase {
    
    private final double positionScalar = 4;
    private final WPI_TalonFX talonFX = new WPI_TalonFX(Gripper.motorID);

    public GripperSubsystem() {
        configureMotor(talonFX);
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(0, 0);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Drive.timeoutMs);
        talon.config_kP(0, Gripper.kP);
        talon.config_kI(0, Gripper.kI);
        talon.config_kD(0, Gripper.kD);
        talon.config_kF(0, Gripper.kF);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Gripper.maxSpeed);
    }

    public void setPower(double power) {
        talonFX.set(power);
    }

    /**
     * @return Grippers percent being closed, 1 is fully closed, 0 is fully open
     */
    public double getPercentClosed() { 
        return talonFX.getSelectedSensorPosition() / Gripper.encoderUnitsPerRev / positionScalar;
    }

    /**
     * Sets percent closed of the gripper, 1 is fully closed, 0 is fully open
     * @param percent
     */
    public void setPercentClosed(double percent) {
        if (percent < 0 || percent > 1) return;
        double targetPosition = percent * positionScalar * Gripper.encoderUnitsPerRev;
        talonFX.set(TalonFXControlMode.Position, targetPosition);
    }
    
    public boolean pollLimitSwitch() {
        return talonFX.isFwdLimitSwitchClosed() == 1;
    }

    public void calibrate(double pos) {
        talonFX.setSelectedSensorPosition(pos);
    }

}

