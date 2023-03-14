package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Lib.Encoder;
import frc.robot.Lib.util.Util;


public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonFX talonFXL = new WPI_TalonFX(Drive.LID); 
    private final WPI_TalonFX talonFXLfollow = new WPI_TalonFX(Drive.LFID);  
    private final WPI_TalonFX talonFXR = new WPI_TalonFX(Drive.RID); 
    private final WPI_TalonFX talonFXRfollow = new WPI_TalonFX(Drive.RFID);
    private final DifferentialDriveOdometry odometry;
    private final PigeonIMU pigeon;

    public DriveSubsystem(PigeonIMU pigeon) {
        this.pigeon = pigeon;
        configureMotor(talonFXL);
        configureMotor(talonFXR);
        configureMotor(talonFXLfollow);
        configureMotor(talonFXRfollow);
        talonFXR.setInverted(true);
        talonFXRfollow.setInverted(true);
        talonFXLfollow.follow(talonFXL);
        talonFXRfollow.follow(talonFXR);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), getleftPostition(), getRightPostition());
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(0, 0);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
        talon.config_kP(0, Drive.kP);
        talon.config_kI(0, Drive.kI);
        talon.config_kD(0, Drive.kD);
        talon.config_kF(0, Drive.kF);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Drive.maxOutput);
        
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), getleftPostition(), getRightPostition());
    }

    public void setPower(double leftPower, double rightPower) {
        talonFXL.set(TalonFXControlMode.PercentOutput, leftPower);
        talonFXR.set(TalonFXControlMode.PercentOutput, rightPower);
    }

    /**
     * Drives to a set position using Motion Magic. Should configure motion magic params before calling.
     * @param distanceMeters distance to travel in meters
     */
    public void driveDistance(double distanceMeters) {
        talonFXL.setIntegralAccumulator(0);
        talonFXR.setIntegralAccumulator(0);
        double sensorUnits = Encoder.fromDistance(distanceMeters, Drive.encoderUnitsPerRev, Drive.gearboxRatio, Drive.wheelDiameterMeters);
        talonFXL.set(TalonFXControlMode.MotionMagic, (talonFXL.getSelectedSensorPosition()+sensorUnits));
        talonFXR.set(TalonFXControlMode.MotionMagic, (talonFXR.getSelectedSensorPosition()+sensorUnits));
    }

    public void stop() {
        talonFXL.neutralOutput();
        talonFXR.neutralOutput();
    }

    /**
     * Configures motion magic values for next run. If your acceleration is the same value as your velocity
     * then it will take 1 second to reach your velocity. Higher values of acceleration will make it get there faster, 
     * lower values will make it get there slower.
     * @param velocity in meters/second
     * @param acceleration in meters/second^2
     */
    public void configMotionMagic(double velocity, double acceleration) {
        talonFXL.configMotionCruiseVelocity(fromVelocity(velocity));
        talonFXR.configMotionCruiseVelocity(fromVelocity(velocity));
        talonFXL.configMotionAcceleration(fromVelocity(acceleration));
        talonFXR.configMotionAcceleration(fromVelocity(acceleration));
    }

    public void calibrate(double pos) {
        talonFXL.setSelectedSensorPosition(pos);
        talonFXR.setSelectedSensorPosition(pos);
    }

    /**
     * @return Distance of left motor in meters
     */
    public double getleftPostition() {
        return Encoder.toDistance(talonFXL.getSelectedSensorPosition(), Drive.encoderUnitsPerRev, Drive.gearboxRatio, Drive.wheelDiameterMeters); 
    }

    /**
     * @return Distance of right motor in meters
     */
    public double getRightPostition() {
        return Encoder.toDistance(talonFXR.getSelectedSensorPosition(), Drive.encoderUnitsPerRev, Drive.gearboxRatio, Drive.wheelDiameterMeters); 
    }

    public double getTarget() {
        return Encoder.toDistance(talonFXL.getClosedLoopTarget(), Drive.encoderUnitsPerRev, Drive.gearboxRatio, Drive.wheelDiameterMeters);
    }

    /**
     * Put motors in brake mode
     */
    public void brake() {
        talonFXL.setNeutralMode(NeutralMode.Brake);
        talonFXLfollow.setNeutralMode(NeutralMode.Brake);
        talonFXR.setNeutralMode(NeutralMode.Brake);
        talonFXRfollow.setNeutralMode(NeutralMode.Brake);
    }

    public void setVoltage(double leftVolts, double rightVolts) {
        talonFXL.setVoltage(leftVolts);
        talonFXR.setVoltage(rightVolts);
    }

    /**
     * @return wheel speeds in meters/second
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(toVelocity(talonFXL.getSelectedSensorVelocity()), toVelocity(talonFXR.getSelectedSensorVelocity()));
    }
    
    /**
     * Sets the velocity of the motors
     * @param leftVelocity The velocity of the left motors in meters/second
     * @param rightVelocity The velocity of the right motors in meters/second
     */
    public void setVelocity(double leftVelocity, double rightVelocity) {
        talonFXL.set(ControlMode.Velocity, fromVelocity(leftVelocity));
        talonFXR.set(ControlMode.Velocity, fromVelocity(rightVelocity));
    }

    /**
     * Sets the velocity of the motor in encoder units
     * @param leftVelocity Velocity of the left motor in encoder units per 100 ms
     * @param rightVelocity Velocity of the right motor in encoder units per 100 ms
     */
    public void setEncoderVelocity(double leftVelocity, double rightVelocity) {
        talonFXL.set(TalonFXControlMode.Velocity, leftVelocity);
        talonFXR.set(TalonFXControlMode.Velocity, rightVelocity);
    }

    /**
     * @param velocity in encoder units/100 ms
     * @return velocity in meters/sec
     */
    private double toVelocity(double velocity) {
        return Encoder.toVelocity(velocity, Drive.encoderUnitsPerRev, Drive.gearboxRatio, Drive.wheelDiameterMeters);
    }

    /**
     * @param velocity in meters/sec
     * @return velocity in encoder units/100 ms
     */
    private double fromVelocity(double velocity) {
        return Encoder.fromVelocity(velocity, Drive.encoderUnitsPerRev, Drive.gearboxRatio, Drive.wheelDiameterMeters);
    }

    /**
     * @return odometry's current pose in meters
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Set the odometry's position 
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getleftPostition(), getRightPostition(), pose);
    }

    /**
     * @return pigeon's heading in degrees
     */
    public double getHeading() {
        return Util.boundedAngleDegrees(pigeon.getFusedHeading());
    }

}
