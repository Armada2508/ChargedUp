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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Gripper;

public class GripperSubsystem extends SubsystemBase {
    
    private boolean calibrated = false;
    private double desiredPosition = 1;
    // private double armOffset = 0;
    // private double wristOffset = 0;
    private final SlewRateLimiter limiter = new SlewRateLimiter(3);
    private final double revolutionsToClosed = 27;
    private final WPI_TalonFX talonFX = new WPI_TalonFX(Gripper.motorID);
    private final ArmSubsystem armSubsystem;
    private final WristSubsystem wristSubsystem;

    public GripperSubsystem(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        configureMotor(talonFX);
        updateArmOffset(-armSubsystem.getSensorPosition());
        updateWristOffset(-wristSubsystem.getSensorPosition());
    }

    @Override
    public void periodic() {
        // if (pollLimitSwitch() && calibrated) {
        //     setPower(-0.05);
        //     desiredPosition = toPosition(talonFX.getSelectedSensorPosition());
        //     System.out.println(talonFX.getSelectedSensorPosition() + ", " + desiredPosition);
        //     return;
        // } 
        // Gripper Compensation
        if (calibrated) {
            double arm = armSubsystem.getSensorPosition();
            double wrist = wristSubsystem.getSensorPosition();
            double pos = limiter.calculate(desiredPosition);
            talonFX.setSelectedSensorPosition(talonFX.getSelectedSensorPosition() - (arm * Gripper.armSensorMultiplier));
            talonFX.setSelectedSensorPosition(talonFX.getSelectedSensorPosition() - (wrist * Gripper.wristSensorMultiplier));
            // System.out.println(wrist + " " + wristOffset + " " + (wrist + wristOffset));
            // System.out.println(fromPosition(pos) + ((arm + armOffset) * Gripper.armSensorMultiplier) + ((wrist + wristOffset) * Gripper.wristSensorMultiplier));
            // System.out.println("Desired Position: " + desiredPosition + " Limited Position: " + pos + " Arm: " + arm + " Wrist: " + wrist + " ArmOffset: " + armOffset + " WristOffset: " + wristOffset);
            // talonFX.set(TalonFXControlMode.Position, fromPosition(pos) + ((arm + armOffset) * Gripper.armSensorMultiplier) + ((wrist + wristOffset) * Gripper.wristSensorMultiplier));  
            // talonFX.set(TalonFXControlMode.Position, fromPosition(pos) + (arm * Gripper.armSensorMultiplier) + (wrist * Gripper.wristSensorMultiplier));
            talonFX.set(TalonFXControlMode.Position, fromPosition(pos));  
        }
        // System.out.println(Math.abs(toPosition(talonFX.getSelectedSensorVelocity())) * 10 + " " + Gripper.maxVelocity);
        // Velocity Check
        if (Math.abs(toPosition(talonFX.getSelectedSensorVelocity())) * 10 > Gripper.maxVelocity) {
            System.out.println("Gripper: HOLY POOP SLOW DOWN");
            if (this.getCurrentCommand() != null) {
                this.getCurrentCommand().cancel();
            }
            talonFX.neutralOutput(); 
        }
    }

    public void updateArmOffset(double offsetSensorUnits) {
        // armOffset += offsetSensorUnits;
        talonFX.setSelectedSensorPosition(talonFX.getSelectedSensorPosition() - (offsetSensorUnits * Gripper.armSensorMultiplier));
    }

    public void updateWristOffset(double offsetSensorUnits) {
        // wristOffset += offsetSensorUnits;
        talonFX.setSelectedSensorPosition(talonFX.getSelectedSensorPosition() - (offsetSensorUnits * Gripper.wristSensorMultiplier));
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
        return (position * revolutionsToClosed * Gripper.encoderUnitsPerRev);
    }

    public double fromVelocity(double velocity) {
        return fromPosition(velocity) * 0.1;
    }

    public boolean pollLimitSwitch() {
        return talonFX.isFwdLimitSwitchClosed() == 1;
    }

    private void setSensor(double pos) {
        talonFX.setSelectedSensorPosition(pos);
    }

    private void startCalibrate() {
        System.out.println("Started Gripper Calibration.");
        desiredPosition = Gripper.max;
        limiter.reset(Gripper.max);
        calibrated = false;
        talonFX.neutralOutput();
    }

    private void endCalibrate() {
        calibrated = true;
        System.out.println("Ended Gripper Calibration.");
    }

    public Command getCalibrateSequence() {
        return new SequentialCommandGroup(
            new InstantCommand(this::startCalibrate, this),
            new ConditionalCommand(new SequentialCommandGroup(
                new InstantCommand(() -> setPower(-0.1)),
                new WaitUntilCommand(() -> !pollLimitSwitch())
            ), new InstantCommand(), this::pollLimitSwitch),
            calibrateGripper(),
            new InstantCommand(this::endCalibrate, this)
        ).withName("GripperCalibrationSequence");
    }

    private Command calibrateGripper() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setPower(0.12), this),
            new WaitUntilCommand(this::pollLimitSwitch),
            new InstantCommand(() -> {
                stop();
                // setSensor(fromPosition(Gripper.max) + ((armSubsystem.getSensorPosition() + armOffset) * Gripper.armSensorMultiplier) + ((wristSubsystem.getSensorPosition() + wristOffset) * Gripper.wristSensorMultiplier));
                // setSensor(fromPosition(Gripper.max) + (armSubsystem.getSensorPosition() * Gripper.armSensorMultiplier) + (wristSubsystem.getSensorPosition() * Gripper.wristSensorMultiplier));
                setSensor(fromPosition(Gripper.max));
            })
        );
    }

}

