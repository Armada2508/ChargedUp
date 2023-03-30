package frc.robot.subsystems;

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
import frc.robot.lib.Encoder;


public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonFX talonFXL = new WPI_TalonFX(Drive.LID); 
    private final WPI_TalonFX talonFXLfollow = new WPI_TalonFX(Drive.LFID);  
    private final WPI_TalonFX talonFXR = new WPI_TalonFX(Drive.RID); 
    private final WPI_TalonFX talonFXRfollow = new WPI_TalonFX(Drive.RFID);
    private final DifferentialDriveOdometry odometry;
    private final PigeonIMU pigeon;

    public DriveSubsystem(PigeonIMU pigeon) {
        this.pigeon = pigeon;
        calibrate(0);
        configureMotor(talonFXL);
        configureMotor(talonFXR);
        configureMotor(talonFXLfollow);
        configureMotor(talonFXRfollow);
        talonFXR.setInverted(true);
        talonFXRfollow.setInverted(true);
        talonFXLfollow.follow(talonFXL);
        talonFXRfollow.follow(talonFXR);
        talonFXL.configNominalOutputForward(Drive.nominalOutputLeft);
        talonFXL.configNominalOutputReverse(-Drive.nominalOutputLeft);
        talonFXR.configNominalOutputForward(Drive.nominalOutputRight);
        talonFXR.configNominalOutputReverse(-Drive.nominalOutputRight);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), getLeftPostition(), getRightPostition());
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(0, 0);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.timeoutMs);
        talon.config_kP(Drive.motionMagicSlot, Drive.motionMagickP);
        talon.config_kI(Drive.motionMagicSlot, Drive.motionMagickI);
        talon.config_kD(Drive.motionMagicSlot, Drive.motionMagickD);
        talon.config_kF(Drive.motionMagicSlot, Drive.motionMagickF);
        talon.config_kP(Drive.velocitySlot, Drive.velocitykP);
        talon.config_kI(Drive.velocitySlot, Drive.velocitykI);
        talon.config_kD(Drive.velocitySlot, Drive.velocitykD);
        talon.config_kF(Drive.velocitySlot, Drive.velocitykF);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Drive.maxOutput);
        talon.setIntegralAccumulator(0);
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftPostition(), getRightPostition());
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
        talonFXL.selectProfileSlot(Drive.motionMagicSlot, 0);
        talonFXR.selectProfileSlot(Drive.motionMagicSlot, 0);
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

    public void holdPosition() {
        talonFXL.selectProfileSlot(Drive.motionMagicSlot, 0);
        talonFXR.selectProfileSlot(Drive.motionMagicSlot, 0);
        configMotionMagic(0.25, 0.25);
        talonFXL.set(TalonFXControlMode.MotionMagic, talonFXL.getSelectedSensorPosition());
        talonFXR.set(TalonFXControlMode.MotionMagic, talonFXR.getSelectedSensorPosition());
    }

    public void calibrate(double pos) {
        talonFXL.setSelectedSensorPosition(pos);
        talonFXR.setSelectedSensorPosition(pos);
    }

    /**
     * @return Distance of left motor in meters
     */
    public double getLeftPostition() {
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

    public double getLeftVelocity() {
        return toVelocity(talonFXL.getSelectedSensorVelocity());
    }

    public double getRightVelocity() {
        return toVelocity(talonFXR.getSelectedSensorVelocity());
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
        talonFXL.selectProfileSlot(Drive.velocitySlot, 0);
        talonFXR.selectProfileSlot(Drive.velocitySlot, 0);
        talonFXL.set(TalonFXControlMode.Velocity, fromVelocity(leftVelocity));
        talonFXR.set(TalonFXControlMode.Velocity, fromVelocity(rightVelocity));
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
    public double fromVelocity(double velocity) {
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
        odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getLeftPostition(), getRightPostition(), pose);
    }

    /**
     * @return pigeon's heading in degrees
     */
    public double getHeading() {
        return 360-pigeon.getFusedHeading();
    }

}
