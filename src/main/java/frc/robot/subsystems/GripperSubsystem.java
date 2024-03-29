package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Gripper;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.util.Util;

public class GripperSubsystem extends SubsystemBase implements Loggable {
    
    private boolean calibrated = false;
    private double desiredPosition = 1;
    private double armOffset = 0;
    private double wristOffset = 0;
    private final SlewRateLimiter limiter = new SlewRateLimiter(Gripper.slewRate);
    private final double revolutionsToOpen = 35;
    private final WPI_TalonFX talonFX = new WPI_TalonFX(Gripper.motorID);
    private final ArmSubsystem armSubsystem;
    private final WristSubsystem wristSubsystem;

    public GripperSubsystem(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        NTLogger.register(this);
        configureMotor(talonFX);
        updateArmOffset(-armSubsystem.getSensorPosition());
        updateWristOffset(-wristSubsystem.getSensorPosition());
    }

    @Override
    public void periodic() {
        if (pollLimitSwitch() && calibrated) {
            setPower(-0.15);
            desiredPosition = toPosition(talonFX.getSelectedSensorPosition());
            return;
        } 
        // Gripper Compensation
        if (calibrated) {
            double arm = armSubsystem.getSensorPosition();
            double wrist = wristSubsystem.getSensorPosition();
            double pos = limiter.calculate(desiredPosition);
            talonFX.set(TalonFXControlMode.Position, fromPosition(pos) + ((arm + armOffset) * Gripper.armSensorMultiplier) + ((wrist + wristOffset) * Gripper.wristSensorMultiplier));  
        }
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Calibrated", calibrated);
        map.put("Desired Position", desiredPosition);
        map.put("Arm Offset", armOffset);
        map.put("Wrist Offset", wristOffset);
        return Util.mergeMaps(map, NTLogger.getTalonLog(talonFX));
    }

    public void updateArmOffset(double offsetSensorUnits) {
        armOffset += offsetSensorUnits;
    }

    public void updateWristOffset(double offsetSensorUnits) {
        wristOffset += offsetSensorUnits;
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(0, 0);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
        talon.config_kP(0, Gripper.kP);
        talon.config_kI(0, Gripper.kI);
        talon.config_kD(0, Gripper.kD);
        talon.config_kF(0, Gripper.kF);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Gripper.maxOutput);
        talon.configNominalOutputForward(Gripper.minOutput);
        talon.configNominalOutputReverse(Gripper.minOutput);
        talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    }

    /**
     * @param power to set the motor between -1.0 and 1.0
     */
    public void setPower(double power) {
        talonFX.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Sets position of the gripper, 1 is fully closed, 0 is fully open
     * @param percent
     */
    public void setPosition(double position) {
        if (!calibrated) return;
        if (position > Gripper.grabCone) position = Gripper.grabCone;
        if (position < Gripper.open) position = Gripper.open;
        desiredPosition = position;
    }

    public void stop() {
        talonFX.neutralOutput();
    }

    /**
     * 1 is on limit switch, 0 is fully open
     */
    public double getPhysicalPosition() {
        return toPosition(talonFX.getSelectedSensorPosition());
    }

    public double getPhysicalTarget() {
        return desiredPosition;
    }

    public double getSensorPositon() { 
        return talonFX.getSelectedSensorPosition();
    }

    public double getTarget() {
        return talonFX.getClosedLoopTarget();
    }

    public double toPosition(double sensorPos) {
        return (sensorPos
        - ((armSubsystem.getSensorPosition() + armOffset) * Gripper.armSensorMultiplier)
        - ((wristSubsystem.getSensorPosition() + wristOffset) * Gripper.wristSensorMultiplier))
        / (Gripper.encoderUnitsPerRev * revolutionsToOpen);    
    }

    private double fromPosition(double position) {
        return (position * revolutionsToOpen * Gripper.encoderUnitsPerRev);
    }

    public boolean pollLimitSwitch() {
        return talonFX.isFwdLimitSwitchClosed() == 0;
    }

    public boolean isCalibrated() {
        return calibrated;
    }

    private void setSensor(double pos) {
        talonFX.setSelectedSensorPosition(pos);
    }

    private void startCalibrate() {
        System.out.println("Started Gripper Calibration.");
        calibrated = false;
        talonFX.neutralOutput();
    }

    private void endCalibrate(double pos) {
        desiredPosition = pos;
        limiter.reset(pos);
        calibrated = true;
        System.out.println("Ended Gripper Calibration.");
    }

    public Command getCalibrateSequence() {
        return getCalibrateSequence(Gripper.onLimit, 0.1);
    }

    public Command getCalibrateSequence(double zeroPos, double power) {
        return new SequentialCommandGroup(
            new InstantCommand(this::startCalibrate, this),
            new ConditionalCommand(new SequentialCommandGroup(
                new InstantCommand(() -> setPower(-Math.abs(power))), // Get off limit switch
                new WaitUntilCommand(() -> !pollLimitSwitch())
            ), new InstantCommand(), this::pollLimitSwitch),
            calibrateGripper(zeroPos, power),
            new InstantCommand(() -> endCalibrate(zeroPos), this)
        ).withName("GripperCalibrationSequence");
    }

    private Command calibrateGripper(double zeroPos, double power) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setPower(power), this),
            new WaitUntilCommand(this::pollLimitSwitch),
            new InstantCommand(() -> {
                stop();
                setSensor(fromPosition(zeroPos) + 
                ((armSubsystem.getSensorPosition() + armOffset) * Gripper.armSensorMultiplier) + 
                ((wristSubsystem.getSensorPosition() + wristOffset) * Gripper.wristSensorMultiplier));            
            })
        );
    }

}

