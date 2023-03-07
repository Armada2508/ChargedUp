package frc.robot.subsystems;

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
import frc.robot.Constants.Gripper;
import frc.robot.commands.Arm.CalibrateGripperCommand;

public class GripperSubsystem extends SubsystemBase {
    
    private boolean calibrated = false;
    private double currentOffset = 0;
    private double lastArm = 0;
    private double lastWrist = 0;
    private boolean moving = true;
    private final double revolutionsToClosed = 1/3;
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
            // talonFX.set(TalonFXControlMode.Position, talonFX.getSelectedSensorPosition() + arm + wrist);    
            lastArm = arm;
            lastWrist = wrist;    
        }
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
        talon.configForwardSoftLimitThreshold(fromPercent(Gripper.max), Constants.timeoutMs);
        talon.configReverseSoftLimitThreshold(fromPercent(Gripper.min), Constants.timeoutMs);
    }

    /**
     * @param power to set the motor between -1.0 and 1.0
     */
    public void setPower(double power) {
        if (!calibrated) return;
        moving = true;
        talonFX.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Sets percent closed of the gripper, 1 is fully closed, 0 is fully open
     * @param percent
     */
    public void setPercentClosed(double percent) {
        if (!calibrated) return;
        if (percent > Gripper.max) percent = Gripper.max;
        if (percent < Gripper.min) percent = Gripper.min;
        moving = true;
        double targetPosition = fromPercent(percent);
        talonFX.set(TalonFXControlMode.MotionMagic, targetPosition);
    }

    /**
     * Configures motion magic values for next run. If your acceleration is the same value as your velocity
     * then it will take 1 second to reach your velocity. Higher values of acceleration will make it get there faster, 
     * lower values will make it get there slower.
     * @param velocity in percent/second
     * @param acceleration in percent/second/second
     */
    public void configMotionMagic(double velocity, double acceleration) {
        talonFX.setIntegralAccumulator(0);
        talonFX.configMotionCruiseVelocity(fromVelocity(velocity));
        talonFX.configMotionAcceleration(fromVelocity(acceleration));
    }

    public void stop() {
        talonFX.neutralOutput();
    }

    /**
     * @return Grippers percent being closed, 1 is fully closed, 0 is fully open
     */
    public double getPercentClosed() { 
        return toPercent(talonFX.getSelectedSensorPosition());
    }

    public double getTarget() {
        return toPercent(talonFX.getClosedLoopTarget());
    }

    public void finishedMoving() {
        moving = false;
    }

    public double toPercent(double sensorPos) {
        return (((sensorPos - currentOffset) / Gripper.encoderUnitsPerRev) / revolutionsToClosed);
    }

    public double fromPercent(double percent) {
        return (percent * revolutionsToClosed * Gripper.encoderUnitsPerRev) + currentOffset;
    }

    public double fromVelocity(double velocity) {
        return fromPercent(velocity) * 0.1;
    }

    public boolean pollLimitSwitch() {
        return talonFX.isRevLimitSwitchClosed() == 1;
    }

    public void calibrate(double pos) {
        talonFX.setSelectedSensorPosition(pos);
    }

    private void configSoftwareLimits(boolean enable) {
        talonFX.configForwardSoftLimitEnable(enable, Constants.timeoutMs);
        talonFX.configReverseSoftLimitEnable(enable, Constants.timeoutMs);
    }

    private void startCalibrate() {
        calibrated = false;
        talonFX.neutralOutput();
        configSoftwareLimits(false);
    }

    private void endCalibrate() {
        configSoftwareLimits(true);
        calibrated = true;
    }

    public Command getCalibrateSequence() {
        double waitTime = 1;
        return new SequentialCommandGroup(
            new InstantCommand(this::startCalibrate, this),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> talonFX.set(TalonFXControlMode.PercentOutput, 0.1)),
                    new WaitCommand(waitTime)), 
                new InstantCommand(), this::pollLimitSwitch
            ),
            new CalibrateGripperCommand(this),
            new InstantCommand(this::endCalibrate, this)
        );
    }

}

