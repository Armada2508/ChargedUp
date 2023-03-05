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
import frc.robot.Constants.Arm;
import frc.robot.Lib.Encoder;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Arm.CalibrateArmCommand;

public class ArmSubsystem extends SubsystemBase {
    
    private final WPI_TalonFX talonFX = new WPI_TalonFX(Arm.motorID);
    private final WPI_TalonFX talonFXFollow = new WPI_TalonFX(Arm.motorIDFollow);

    public ArmSubsystem() {
        configureMotor(talonFX);
        configureMotor(talonFXFollow);
        talonFXFollow.setInverted(true);
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.setNeutralMode(NeutralMode.Brake);
        talon.selectProfileSlot(0, 0);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
        talon.config_kP(0, Arm.kP);
        talon.config_kI(0, Arm.kI);
        talon.config_kD(0, Arm.kD);
        talon.config_kF(0, Arm.kF);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Arm.maxSpeed);
        talon.configForwardSoftLimitThreshold(fromAngle(Arm.maxDegrees), Constants.timeoutMs);
        talon.configReverseSoftLimitThreshold(fromAngle(Arm.minDegrees), Constants.timeoutMs);
    }

    public void periodic() {
    }

    private void followMotor() {
        talonFXFollow.follow(talonFX);
    }

    /**
     * @param power to set the motor between -1.0 and 1.0
     */
    public void setPower(double power) {
        talonFX.set(TalonFXControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, getFeedForward());
    }

    /**
     * Have arm go to a desired degrees. Must be between min and max degrees. You should call configMotionMagic() before calling this method.
     * @param theta degrees to go to
     */
    public void setPosition(double theta) {
        if (theta > Arm.maxDegrees) theta = Arm.maxDegrees;
        if (theta < Arm.minDegrees) theta = Arm.minDegrees;
        double targetPosition = fromAngle(theta);
        talonFX.set(TalonFXControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, getFeedForward());
    }

    /**
     * @return Arm's current position in degrees
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

    /**
     * 
     * @return Arm's position in degrees when in motion magic mode
     */
    public double getMotionMagicPosition() {
        return toAngle(talonFX.getActiveTrajectoryPosition());
    }

    public void holdPosition() {
        talonFX.set(TalonFXControlMode.Position, talonFX.getSelectedSensorPosition(), DemandType.ArbitraryFeedForward, getFeedForward());
    }

    public double getSensorPosition() {
        return talonFX.getSelectedSensorPosition();
    }

    public double toAngle(double sensorUnits) {
        return Encoder.toRotationalAngle(sensorUnits, Arm.encoderUnitsPerRev, Arm.gearboxRatio);
    }

    public double fromAngle(double theta) {
        return Encoder.fromRotationalAngle(theta, Arm.encoderUnitsPerRev, Arm.gearboxRatio);
    }

    private double fromVelocity(double velocity) {
        return fromAngle(velocity) * 0.1;
    }

    private double getFeedForward() {
        double degrees = getPosition();
        double scalar = Math.sin(Math.toRadians(degrees));
        return Arm.gravityFeedForward * scalar;
    }

    public boolean pollLimitSwitch() {
        return talonFX.isRevLimitSwitchClosed() == 1;
    }

    public void calibrate(double pos) {
        talonFX.setSelectedSensorPosition(pos);
        talonFXFollow.setSelectedSensorPosition(pos);
    }

    public void configSoftwareLimits(boolean enable) {
        talonFX.configForwardSoftLimitEnable(enable, Constants.timeoutMs);
        talonFX.configReverseSoftLimitEnable(enable, Constants.timeoutMs);
        talonFXFollow.configForwardSoftLimitEnable(enable, Constants.timeoutMs);
        talonFXFollow.configReverseSoftLimitEnable(enable, Constants.timeoutMs);
    }

    public Command getCalibrateSequence() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> talonFXFollow.set(TalonFXControlMode.PercentOutput, 0)),
            new InstantCommand(() -> configSoftwareLimits(false), this),
            new CalibrateArmCommand(this, talonFX),
            new ArmCommand(0, this),
            new CalibrateArmCommand(this, talonFXFollow),
            new InstantCommand(this::followMotor, this),
            new InstantCommand(() -> configSoftwareLimits(true), this)
        );
    }

}
