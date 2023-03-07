package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Lib.Encoder;
import frc.robot.commands.Arm.CalibrateArmCommand;

public class ArmSubsystem extends SubsystemBase {
    
    private boolean calibrated = false;
    private final WPI_TalonFX talonFX = new WPI_TalonFX(Arm.motorID);
    private final WPI_TalonFX talonFXFollow = new WPI_TalonFX(Arm.motorIDFollow);

    public ArmSubsystem() {
        configureMotor(talonFX);
        configureMotor(talonFXFollow);
        talonFXFollow.setInverted(true);
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(0, 0);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
        talon.config_kP(Arm.moveSlot, Arm.kPMove);
        talon.config_kI(Arm.moveSlot, Arm.kIMove);
        talon.config_kD(Arm.moveSlot, Arm.kDMove);
        talon.config_kF(Arm.moveSlot, Arm.kFMove);
        talon.config_kP(Arm.holdSlot, Arm.kPHold);
        talon.config_kI(Arm.holdSlot, Arm.kIHold);
        talon.config_kD(Arm.holdSlot, Arm.kDHold);
        talon.config_kF(Arm.holdSlot, Arm.kFHold);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Arm.maxOutput);
        talon.configNominalOutputForward(Arm.minOutput);
        talon.configNominalOutputReverse(Arm.minOutput);
        talon.configForwardSoftLimitThreshold(fromAngle(Arm.maxDegrees - Arm.limitMargin), Constants.timeoutMs);
        talon.configReverseSoftLimitThreshold(fromAngle(Arm.minDegrees + Arm.limitMargin), Constants.timeoutMs);
        talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    }

    public void periodic() {
        // System.out.println(pollLimitSwitch());
        double left = toAngle(talonFX.getSelectedSensorPosition());
        double right = toAngle(talonFXFollow.getSelectedSensorPosition());
        System.out.println(Math.abs(right - left));
        // SlotConfiguration slot = new SlotConfiguration();
        // talonFX.getSlotConfigs(slot, Arm.holdSlot, 50);
        // System.out.println(slot.kP);
        // System.out.println(toAngle(talonFX.getClosedLoopTarget()) + " " + getPosition());
        if ((Math.abs(right - left) > Arm.allowedMotorDifference) && calibrated) {
            // if (this.getCurrentCommand() != null) {
            //     this.getCurrentCommand().cancel();
            // }
            // talonFX.neutralOutput(); 
        }
    //    System.out.println(Math.abs(toAngle(talonFX.getSelectedSensorVelocity() * 10)) + " " + Arm.maxVelocity);
    //    System.out.println(getPosition());
        if (Math.abs(toAngle(talonFX.getSelectedSensorVelocity())) * 10 > Arm.maxVelocity) {
            System.out.println("HOLY POOP SLOW DOWN");
            if (this.getCurrentCommand() != null) {
                this.getCurrentCommand().cancel();
            }
            talonFX.neutralOutput(); 
        } 
    } 

    /**
     * @param power to set the motor between -1.0 and 1.0
     */
    public void setPower(double power) {
        if (!calibrated) return;
        talonFX.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Have arm go to a desired degrees. Must be between min and max degrees. You should call configMotionMagic() before calling this method.
     * @param theta degrees to go to
     */
    public void setPosition(double theta) {
        // System.out.println("Setting " + calibrated);
        if (!calibrated) return;
        if (theta > Arm.maxDegrees - Arm.limitMargin) theta = Arm.maxDegrees - Arm.limitMargin;
        if (theta < Arm.minDegrees + Arm.limitMargin) theta = Arm.minDegrees + Arm.limitMargin;
        talonFX.selectProfileSlot(Arm.moveSlot, 0);
        double targetPosition = fromAngle(theta);
        talonFX.set(TalonFXControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, getFeedForward(theta));
    }

    public void holdPosition() {
        if (!calibrated) return;
        talonFX.selectProfileSlot(Arm.holdSlot, 0);
        talonFX.set(TalonFXControlMode.Position, talonFX.getClosedLoopTarget(), DemandType.ArbitraryFeedForward, getFeedForward(getPosition()));
    }

    public void stop() {
        talonFX.neutralOutput();
    }

    public void disabled() {
        stop();
        talonFX.setNeutralMode(NeutralMode.Coast);
        talonFXFollow.setNeutralMode(NeutralMode.Coast);
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

    private double getFeedForward(double degrees) {
        double scalar = Math.sin(Math.toRadians(degrees));
        System.out.println(Arm.gravityFeedForward * scalar);
        return Arm.gravityFeedForward * scalar;
    }

    public boolean pollLimitSwitch() {
        return talonFX.isRevLimitSwitchClosed() == 0;
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

    private void startCalibrate() {
        calibrated = false;
        talonFX.setNeutralMode(NeutralMode.Coast);
        talonFXFollow.setNeutralMode(NeutralMode.Coast);
        talonFX.neutralOutput();
        talonFXFollow.neutralOutput();
        configSoftwareLimits(false);
    }

    private void endCalibrate() {
        talonFXFollow.follow(talonFX);
        configSoftwareLimits(true);
        talonFX.setNeutralMode(NeutralMode.Brake);
        talonFXFollow.setNeutralMode(NeutralMode.Brake);
        calibrated = true;
    }

    public Command getCalibrateSequence() {
        double waitTime = 1;
        return new SequentialCommandGroup(
            new InstantCommand(this::startCalibrate, this),
            new InstantCommand(() -> talonFX.set(TalonFXControlMode.PercentOutput, 0.05)),
            new WaitCommand(waitTime), 
            new CalibrateArmCommand(this, talonFX),
            new InstantCommand(() -> talonFXFollow.set(TalonFXControlMode.PercentOutput, 0.04)),
            new WaitCommand(waitTime),
            new CalibrateArmCommand(this, talonFXFollow),
            new InstantCommand(this::endCalibrate, this)
        );
    }

}
