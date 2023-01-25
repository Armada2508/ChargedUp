package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Wrist;

public class WristSubsystem extends SubsystemBase {

    private final double gravityConstant = 0.07;
    private WPI_TalonFX talonFX = new WPI_TalonFX(Wrist.motorID);

    /**
     * 
     * @param power to set the motor between -1.0 and 1.0
     */
    public void setPower(double power) {
        talonFX.set(TalonFXControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, getFeedForward());
    }

    /**
     * Have Wrist go to a desired degrees. Must be between min and max degrees.
     * @param theta degrees to go to
     */
    public void setPosition(double theta) {
        if (theta > Wrist.maxDegrees || theta < Wrist.minDegrees) return;
        double targetPosition = theta / Wrist.degreesPerEncoderUnit;
        talonFX.set(TalonFXControlMode.Position, targetPosition, DemandType.ArbitraryFeedForward, getFeedForward());
    }

    /**
     * 
     * @return Wrist's current position in degrees
     */
    public double getPosition() {
        return talonFX.getSelectedSensorPosition() * Wrist.degreesPerEncoderUnit;
    }
    
    private double getFeedForward() {
        double degrees = getPosition();
        double scalar = Math.cos(Math.toRadians(degrees));
        return gravityConstant * scalar;
    }

    public void calibrate() {
        talonFX.setSelectedSensorPosition(0);
    }

}
