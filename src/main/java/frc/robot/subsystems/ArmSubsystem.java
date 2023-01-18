package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;

public class ArmSubsystem extends SubsystemBase {
    
    private WPI_TalonFX talonFX = new WPI_TalonFX(Arm.motorID);
    private DigitalInput limitSwitch = new DigitalInput(0);

    public ArmSubsystem() {
    }

    @Override
    public void periodic() {}

    /**
     * 
     * @param power to set the motor between -1.0 and 1.0
     */
    public void setPower(double power) {
        talonFX.set(power);
    }

    /**
     * Have arm go to a desired degrees. Must be between min and max degrees.
     * @param theta degrees to go to
     */
    public void setArmPosition(double theta) {
        if (theta > Arm.maxDegrees || theta < Arm.minDegrees) return;
        double targetPosition = theta / Arm.degreesPerEncoderUnit;
        talonFX.set(TalonFXControlMode.Position, targetPosition);
    }

    /**
     * 
     * @return Arm's current position in degrees
     */
    public double getArmPosition() {
        return talonFX.getSelectedSensorPosition() * Arm.degreesPerEncoderUnit;
    }

    public boolean pollLimitSwitch() {
        return !limitSwitch.get(); // Switches are held high
    }

    public void stop() {
        talonFX.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void calibrate() {
        talonFX.setSelectedSensorPosition(0);
    }

}
