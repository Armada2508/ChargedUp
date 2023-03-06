package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Wrist;
import frc.robot.Lib.Encoder;
import frc.robot.commands.Arm.CalibrateWristCommand;

public class WristSubsystem extends SubsystemBase {

    private final WPI_TalonFX talonFX = new WPI_TalonFX(Wrist.motorID);

    public WristSubsystem() {
       configureMotor(talonFX);
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.setNeutralMode(NeutralMode.Brake);
        talon.selectProfileSlot(0, 0);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
        talon.config_kP(0, Wrist.kP);
        talon.config_kI(0, Wrist.kI);
        talon.config_kD(0, Wrist.kD);
        talon.config_kF(0, Wrist.kF);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Wrist.maxSpeed);
        talon.configNominalOutputForward(Wrist.minSpeed);
        talon.configNominalOutputReverse(Wrist.minSpeed);
        talon.configForwardSoftLimitThreshold(fromAngle(Wrist.maxDegrees), Constants.timeoutMs);
        talon.configReverseSoftLimitThreshold(fromAngle(Wrist.minDegrees), Constants.timeoutMs);
    }

    /**
     * @param power to set the motor between -1.0 and 1.0
     */
    public void setPower(double power) {
        talonFX.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Have Wrist go to a desired degrees. Must be between min and max degrees.
     * @param theta degrees to go to
     */
    public void setPosition(double theta) {
        if (theta > Wrist.maxDegrees) theta = Wrist.maxDegrees;
        if (theta < Wrist.minDegrees) theta = Wrist.minDegrees;
        double targetPosition = fromAngle(theta);
        talonFX.set(TalonFXControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, getFeedForward());
    }

    /**
     * @return Wrist's current position in degrees
     */
    public double getPosition() {
        return toAngle(talonFX.getSelectedSensorPosition());
    }
    
    /**
     * Configures motion magic values for next run. If your acceleration is the same value as your velocity
     * then it will take 1 second to reach your velocity. Higher values of acceleration will make it get there faster, 
     * lower values will make it get there slower.
     * @param velocity in degrees/second
     * @param acceleration in degrees/second/second
     */
    public void configMotionMagic(double velocity, double acceleration) {
        talonFX.setIntegralAccumulator(0);
        talonFX.configMotionCruiseVelocity(fromVelocity(velocity));
        talonFX.configMotionAcceleration(fromVelocity(acceleration));
    }

    public double getMotionMagicPosition() {
        return toAngle(talonFX.getActiveTrajectoryPosition());
    }

    public void holdPosition() {
        talonFX.set(TalonFXControlMode.Position, talonFX.getSelectedSensorPosition());
    }

    public double getSensorPosition() {
        return talonFX.getSelectedSensorPosition();
    }

    private double toAngle(double sensorUnits) {
        return Encoder.toRotationalAngle(sensorUnits, Wrist.encoderUnitsPerRev, Wrist.pulleyRatio);
    }

    private double fromAngle(double theta) {
        return Encoder.fromRotationalAngle(theta, Wrist.encoderUnitsPerRev, Wrist.pulleyRatio);
    }

    private double fromVelocity(double velocity) {
        return fromAngle(velocity) * 0.1;
    }
    
    private double getFeedForward() {
        double degrees = getPosition();
        double scalar = Math.cos(Math.toRadians(degrees));
        return Wrist.gravityFeedForward * scalar;
    }

    public boolean pollLimitSwitch() {
        return talonFX.isRevLimitSwitchClosed() == 1;
    }

    public void calibrate(double pos) {
        talonFX.setSelectedSensorPosition(pos);
    }

    public void enableSoftwareLimits() {
        talonFX.configForwardSoftLimitEnable(true, Constants.timeoutMs);
        talonFX.configReverseSoftLimitEnable(true, Constants.timeoutMs);
    }

    public Command getCalibrateSequence() {
        return new SequentialCommandGroup(
            new CalibrateWristCommand(this),
            new InstantCommand(this::enableSoftwareLimits, this)
        );
    }

}
