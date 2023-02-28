package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Gripper;

public class GripperSubsystem extends SubsystemBase {
    
    private double currentOffset = 0;
    private double lastArm = 0;
    private double lastWrist = 0;
    private boolean moving = true;
    private final double revolutionsToClosed = 4;
    private final WPI_TalonFX talonFX = new WPI_TalonFX(Gripper.motorID);
    private final ArmSubsystem armSubsystem;
    private final WristSubsystem wristSubsystem;

    public GripperSubsystem(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        configureMotor(talonFX);
    }

    @Override
    public void periodic() {
        if (!moving) {
            double arm = (armSubsystem.getSensorPosition() - lastArm) * Gripper.armSensorOffset;
            double wrist = (wristSubsystem.getSensorPosition() - lastWrist) * Gripper.wristSensorOffset;
            currentOffset += (arm + wrist);
            talonFX.set(TalonFXControlMode.Position, talonFX.getSelectedSensorPosition() + arm + wrist);        
        }
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(0, 0);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
        talon.config_kP(0, Gripper.kP);
        talon.config_kI(0, Gripper.kI);
        talon.config_kD(0, Gripper.kD);
        talon.config_kF(0, Gripper.kF);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Gripper.maxSpeed);
        talonFX.configForwardSoftLimitThreshold(fromPercent(Gripper.max), Constants.timeoutMs);
        talonFX.configReverseSoftLimitThreshold(fromPercent(Gripper.min), Constants.timeoutMs);
        talonFX.configForwardSoftLimitEnable(true, Constants.timeoutMs);
        talonFX.configReverseSoftLimitEnable(true, Constants.timeoutMs);
    }

    /**
     * @param power to set the motor between -1.0 and 1.0
     */
    public void setPower(double power) {
        moving = true;
        talonFX.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Sets percent closed of the gripper, 1 is fully closed, 0 is fully open
     * @param percent
     */
    public void setPercentClosed(double percent) {
        if (percent < 0 || percent > 1) return;
        moving = true;
        double targetPosition = fromPercent(percent);
        talonFX.set(TalonFXControlMode.MotionMagic, targetPosition);
    }

    /**
     * @return Grippers percent being closed, 1 is fully closed, 0 is fully open
     */
    public double getPercentClosed() { 
        return toPercent(talonFX.getSelectedSensorPosition());
    }

    public double getMotionMagicPosition() {
        return toPercent(talonFX.getActiveTrajectoryPosition());
    }

    public void finishedMoving() {
        moving = false;
    }

    private double toPercent(double sensorPos) {
        return (((sensorPos - currentOffset) / Gripper.encoderUnitsPerRev) / revolutionsToClosed);
    }

    private double fromPercent(double percent) {
        return (percent * revolutionsToClosed * Gripper.encoderUnitsPerRev) + currentOffset;
    }

    public boolean pollLimitSwitch() {
        return talonFX.isFwdLimitSwitchClosed() == 1;
    }

    public void calibrate(double pos) {
        talonFX.setSelectedSensorPosition(pos);
    }

}

