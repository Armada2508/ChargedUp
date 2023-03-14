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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Wrist;
import frc.robot.Lib.Encoder;

public class WristSubsystem extends SubsystemBase {

    private boolean calibrated = false;
    private final WPI_TalonFX talonFX = new WPI_TalonFX(Wrist.motorID);

    public WristSubsystem() {
       configureMotor(talonFX);
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(0, 0);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
        talon.config_kP(0, Wrist.kP);
        talon.config_kI(0, Wrist.kI);
        talon.config_kD(0, Wrist.kD);
        talon.config_kF(0, Wrist.kF);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Wrist.maxOutput);
        talon.configNominalOutputForward(Wrist.minOutput);
        talon.configNominalOutputReverse(Wrist.minOutput);
        talon.configForwardSoftLimitThreshold(fromAngle(Wrist.maxDegrees), Constants.timeoutMs);
        talon.configReverseSoftLimitThreshold(fromAngle(Wrist.minDegrees), Constants.timeoutMs);
    }
    
    @Override
    public void periodic() {
        // System.out.println(pollLimitSwitch());
        // if (this.getCurrentCommand() != null) {
            // System.out.println(this.getCurrentCommand().getName());
        // }
    }

    /**
     * @param power to set the motor between -1.0 and 1.0
     */
    public void setPower(double power) {
        if (!calibrated) return;
        talonFX.set(TalonFXControlMode.PercentOutput, power);
    }

    /**
     * Have Wrist go to a desired degrees. Must be between min and max degrees.
     * @param theta degrees to go to
     */
    public void setPosition(double theta) {
        if (!calibrated) return;
        if (theta > Wrist.maxDegrees) theta = Wrist.maxDegrees;
        if (theta < Wrist.minDegrees) theta = Wrist.minDegrees;
        double targetPosition = fromAngle(theta);
        talonFX.set(TalonFXControlMode.MotionMagic, targetPosition);
    }

    public void stop() {
        talonFX.neutralOutput();
    }

    /**
     * @return Wrist's current position in degrees
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
        return Encoder.toRotationalAngle(sensorUnits, Wrist.encoderUnitsPerRev, Wrist.movementRatio);
    }

    public double fromAngle(double theta) {
        return Encoder.fromRotationalAngle(theta, Wrist.encoderUnitsPerRev, Wrist.movementRatio);
    }

    public double fromVelocity(double velocity) {
        return fromAngle(velocity) * 0.1;
    }
    
    public boolean pollLimitSwitch() {
        return talonFX.isFwdLimitSwitchClosed() == 1;
    }

    private void setSensor(double pos) {
        talonFX.setSelectedSensorPosition(pos);
    }

    private void configSoftwareLimits(boolean enable) {
        talonFX.configForwardSoftLimitEnable(enable, Constants.timeoutMs);
        talonFX.configReverseSoftLimitEnable(enable, Constants.timeoutMs);
    }

    private void startCalibrate() {
        System.out.println("Started Wrist Calibration.");
        calibrated = false;
        talonFX.neutralOutput();
        configSoftwareLimits(false);
    }

    private void endCalibrate() {
        configSoftwareLimits(true);
        calibrated = true;
        System.out.println("Ended Wrist Calibration.");
    }

    public Command getCalibrateSequence(GripperSubsystem gripperSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(this::startCalibrate, this),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    // new PrintCommand("On Wrist Limit Switch."),
                    new InstantCommand(() -> talonFX.set(TalonFXControlMode.PercentOutput, -0.05)),
                    new WaitUntilCommand(() -> !pollLimitSwitch())), 
                new InstantCommand(), this::pollLimitSwitch
            ),
            calibrateWrist(gripperSubsystem),
            new InstantCommand(this::endCalibrate, this)
        );
    }

    private Command calibrateWrist(GripperSubsystem gripperSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> talonFX.set(TalonFXControlMode.PercentOutput, 0.10), this),
            // new PrintCommand("set power to cal wrist"),
            new WaitUntilCommand(this::pollLimitSwitch),
            new InstantCommand(() -> {
                stop();
                double calibrateAngle = Wrist.maxDegrees;
                // System.out.println(getSensorPosition() + " " + fromAngle(calibrateAngle) + " " + (getSensorPosition() - fromAngle(calibrateAngle)));
                gripperSubsystem.updateWristOffset(getSensorPosition() - fromAngle(calibrateAngle));
                setSensor(fromAngle(calibrateAngle));
            })
        );
    }

}
