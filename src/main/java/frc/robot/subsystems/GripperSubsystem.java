package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Gripper;
import frc.robot.commands.Arm.CalibrateGripperCommand;

public class GripperSubsystem extends SubsystemBase {
    
    private boolean calibrated = false;
    private double desiredPosition = 1;
    private double armOffset = 0;
    private double wristOffset = 0;
    private final SlewRateLimiter limiter = new SlewRateLimiter(1);
    private final double revolutionsToClosed = 25;
    private final WPI_TalonFX talonFX = new WPI_TalonFX(Gripper.motorID);
    private final ArmSubsystem armSubsystem;
    private final WristSubsystem wristSubsystem;

    public GripperSubsystem(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        configureMotor(talonFX);
        setArmOffset(-armSubsystem.getSensorPosition());
        setWristOffset(-wristSubsystem.getSensorPosition());
    }

    @Override
    public void periodic() {
        // System.out.println(getPercentClosed());
        // System.out.println(pollLimitSwitch());
        // System.out.println(talonFX.getSelectedSensorPosition());
        // System.out.println(fromPercent(1) + " " + fromVelocity(1) + " " + toPercent(fromPercent(1)));
        // System.out.println(talonFX.getClosedLoopError());
        // System.out.println(fromPosition(0.01));
        if (pollLimitSwitch()) {
            armSubsystem.stop();
            wristSubsystem.stop();
        }
        if (calibrated) {
            double arm = armSubsystem.getSensorPosition();
            double wrist = wristSubsystem.getSensorPosition();
            double pos = limiter.calculate(desiredPosition);
            System.out.println("Desired Position: " + desiredPosition + " Limited Position: " + pos + " Arm: " + arm + " Wrist: " + wrist + " ArmOffset: " + armOffset + " WristOffset: " + wristOffset);
            talonFX.set(TalonFXControlMode.Position, fromPosition(pos) + ((arm + armOffset) * Gripper.armSensorMultiplier) + ((wrist + wristOffset) * Gripper.wristSensorMultiplier));    
        }
    }

    public void setArmOffset(double offset) {
        System.out.println("Arm Offset: " + offset);
        armOffset = offset;
    }

    public void setWristOffset(double offset) {
        wristOffset = offset;
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
        // talon.configForwardSoftLimitThreshold(fromPosition(Gripper.max), Constants.timeoutMs);
        talon.configReverseSoftLimitThreshold(fromPosition(Gripper.min), Constants.timeoutMs);
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
        if (position < Gripper.min) position = Gripper.min;
        desiredPosition = position;
    }

    public void stop() {
        talonFX.neutralOutput();
    }

    /**
     * @return position of the gripper, 1 is fully closed, 0 is fully open
     */
    public double getPosition() { 
        return toPosition(talonFX.getSelectedSensorPosition());
    }

    public double getTarget() {
        return toPosition(talonFX.getClosedLoopTarget());
    }

    public double toPosition(double sensorPos) {
        return ((sensorPos / Gripper.encoderUnitsPerRev) / revolutionsToClosed);
    }

    public double fromPosition(double position) {
        // System.out.println("Pos: " + position + ", RevToClosed: " + revolutionsToClosed + ", EPR: " + Gripper.encoderUnitsPerRev + ", CurrentOffset:" + currentOffset);
        return (position * revolutionsToClosed * Gripper.encoderUnitsPerRev);
    }

    public double fromVelocity(double velocity) {
        return fromPosition(velocity) * 0.1;
    }

    public boolean pollLimitSwitch() {
        return talonFX.isFwdLimitSwitchClosed() == 1;
    }

    public void calibrate(double pos) {
        talonFX.setSelectedSensorPosition(pos);
    }

    private void configSoftwareLimits(boolean enable) {
        // talonFX.configForwardSoftLimitEnable(enable, Constants.timeoutMs);
        talonFX.configReverseSoftLimitEnable(enable, Constants.timeoutMs);
    }

    private void startCalibrate() {
        desiredPosition = 1;
        calibrated = false;
        talonFX.neutralOutput();
        configSoftwareLimits(false);
    }

    private void endCalibrate() {
        configSoftwareLimits(true);
        calibrated = true;
        System.out.println("Ended Gripper Calibration.");
    }

    public Command getCalibrateSequence() {
        return new SequentialCommandGroup(
            new InstantCommand(this::startCalibrate, this),
            new ConditionalCommand(new SequentialCommandGroup(
                new InstantCommand(() -> talonFX.set(TalonFXControlMode.PercentOutput, -0.05)),
                new WaitCommand(1.5)
            ), new InstantCommand(), this::pollLimitSwitch),
            new CalibrateGripperCommand(this),
            new InstantCommand(this::endCalibrate, this)
        );
    }

}

