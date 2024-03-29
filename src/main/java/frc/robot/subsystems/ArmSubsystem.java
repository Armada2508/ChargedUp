package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.commands.arm.CalibrateArmCommand;
import frc.robot.lib.Encoder;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.util.Util;

public class ArmSubsystem extends SubsystemBase implements Loggable {
    
    private boolean calibrated = false;
    private final WPI_TalonFX talonFX = new WPI_TalonFX(Arm.motorID);
    private final WPI_TalonFX talonFXFollow = new WPI_TalonFX(Arm.motorIDFollow);

    public ArmSubsystem() {
        NTLogger.register(this);
        configureMotor(talonFX);
        configureMotor(talonFXFollow);
        talonFXFollow.setInverted(true);
        talonFXFollow.follow(talonFX);
    }

    @Override
    public void periodic() {} 

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Calibrated", calibrated);
        return Util.mergeMaps(map, NTLogger.getTalonLog(talonFX), NTLogger.getTalonLog(talonFXFollow));
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(0, 0);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.config_kP(0, Arm.kP);
        talon.config_kI(0, Arm.kI);
        talon.config_kD(0, Arm.kD);
        talon.config_kF(0, Arm.kF);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Arm.maxOutput);
        talon.configNominalOutputForward(Arm.minOutput);
        talon.configNominalOutputReverse(Arm.minOutput);
        talon.configForwardSoftLimitThreshold(fromAngle(Arm.maxDegrees - Arm.limitMargin), Constants.timeoutMs);
        talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    }

    /**
     * @param power to set the motor between -1.0 and 1.0
     */
    public void setPower(double power) {
        talonFX.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Have arm go to a desired degrees. Must be between min and max degrees. You should call configMotionMagic() before calling this method.
     * @param theta degrees to go to
     */
    public void setPosition(double theta) {
        if (!calibrated) return;
        if (theta > Arm.maxDegrees - Arm.limitMargin) theta = Arm.maxDegrees - Arm.limitMargin;
        if (theta < Arm.minDegrees + Arm.limitMargin) theta = Arm.minDegrees + Arm.limitMargin;
        double targetPosition = fromAngle(theta);
        talonFX.set(TalonFXControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, getFeedForward(theta));
    }

    public void stop() {
        talonFX.neutralOutput();
    }

    /**
     * @return Arm's current position in degrees
     */
    public double getPosition() {
        return toAngle(talonFX.getSelectedSensorPosition());
    }

    public double getTarget() {
        return toAngle(talonFX.getClosedLoopTarget());
    }

    public boolean insideFrame() {
        return getPosition() < Arm.insideFrameDeg;
    }

    /**
     * Configures motion magic values for next run. If your acceleration is the same value as your velocity
     * then it will take 1 second to reach your velocity. Higher values of acceleration will make it get there faster, 
     * lower values will make it get there slower.
     * @param velocity in degrees/second
     * @param acceleration in degrees/second/second
     */
    public void configMotionMagic(double velocity, double acceleration) {
        talonFX.configMotionCruiseVelocity(fromVelocity(velocity));
        talonFX.configMotionAcceleration(fromVelocity(acceleration));
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

    public double fromVelocity(double velocity) {
        return fromAngle(velocity) * 0.1;
    }

    private double getFeedForward(double degrees) {
        double scalar = Math.sin(Math.toRadians(degrees));
        return Arm.gravityFeedForward * scalar;
    }

    public boolean pollLimitSwitch() {
        return talonFX.isRevLimitSwitchClosed() == 0;
    }

    private void configSoftwareLimits(boolean enable) {
        talonFX.configForwardSoftLimitEnable(enable, Constants.timeoutMs);
    }

    private void startCalibrate() {
        System.out.println("Started Arm Calibration.");
        calibrated = false;
        talonFX.neutralOutput();
        configSoftwareLimits(false);
    }

    private void endCalibrate() {
        configSoftwareLimits(true);
        calibrated = true;
        System.out.println("Ended Arm Calibration.");
    }

    public Command getCalibrateSequence(WristSubsystem wristSubsystem, GripperSubsystem gripperSubsystem) {
        double waitTime = .25;
        return new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(this::startCalibrate, this),
                new ConditionalCommand(new SequentialCommandGroup(
                    new InstantCommand(() -> talonFX.set(TalonFXControlMode.PercentOutput, 0.10)),
                    new WaitCommand(waitTime)
                ), new InstantCommand(), this::pollLimitSwitch),
                new CalibrateArmCommand(talonFX, this, gripperSubsystem),
                new InstantCommand(this::endCalibrate, this)
        ), new InstantCommand(), wristSubsystem::pollLimitSwitch).withName("ArmCalibrationSequence");
    }

}
