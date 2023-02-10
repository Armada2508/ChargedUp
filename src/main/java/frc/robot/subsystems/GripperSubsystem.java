package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Gripper;

public class GripperSubsystem extends SubsystemBase {
    
    private final double positionScalar = 4;
    private WPI_TalonFX talonFX = new WPI_TalonFX(Gripper.motorID);
    private DigitalInput limitSwitch = new DigitalInput(Gripper.limitSwitchID);

    public GripperSubsystem() {
        talonFX.config_kP(0, Gripper.kP);
        talonFX.config_kI(0, Gripper.kI);
        talonFX.config_kD(0, Gripper.kD);
    }

    public void setPower(double power) {
        talonFX.set(power);
    }

    /**
     * 
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

    public void calibrate() {
        talonFX.setSelectedSensorPosition(0);
    }

    public boolean pollLimitSwitch() {
        return !limitSwitch.get(); // Switches are held high
    }

}

