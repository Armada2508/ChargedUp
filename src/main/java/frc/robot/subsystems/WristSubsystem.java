package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Wrist;
import frc.robot.Lib.Encoder;

public class WristSubsystem extends SubsystemBase {

    private WPI_TalonFX talonFX = new WPI_TalonFX(Wrist.motorID);
    private DigitalInput limitSwitch = new DigitalInput(Wrist.limitSwitchID);

    public WristSubsystem() {
       configureMotor(talonFX);
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(0, 0);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Drive.timeoutMs);
        talon.config_kP(0, Wrist.kP);
        talon.config_kI(0, Wrist.kI);
        talon.config_kD(0, Wrist.kD);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Wrist.maxSpeed);
    }

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
        double targetPosition = fromAngle(theta);
        talonFX.set(TalonFXControlMode.Position, targetPosition, DemandType.ArbitraryFeedForward, getFeedForward());
    }

    /**
     * 
     * @return Wrist's current position in degrees
     */
    public double getPosition() {
        return toAngle(talonFX.getSelectedSensorPosition());
    }

    public double toAngle(double sensorUnits) {
        return Encoder.toRotationalAngle(sensorUnits, Wrist.encoderUnitsPerRev, Wrist.pulleyRatio);
    }

    public double fromAngle(double theta) {
        return Encoder.fromRotationalAngle(theta, Wrist.encoderUnitsPerRev, Wrist.pulleyRatio);
    }
    
    private double getFeedForward() {
        double degrees = getPosition();
        double scalar = Math.cos(Math.toRadians(degrees));
        return Wrist.gravityFeedForward * scalar;
    }

    public void calibrate() {
        talonFX.setSelectedSensorPosition(Wrist.minDegrees);
    }

    public boolean pollLimitSwitch() {
        return !limitSwitch.get(); // Switches are held high
    }

}
