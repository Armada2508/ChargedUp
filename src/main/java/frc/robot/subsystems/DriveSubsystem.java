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
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.Lib.Encoder;
import frc.robot.Lib.util.Util;


public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonFX TalonFXL = new WPI_TalonFX(Drive.LID); 
    private final WPI_TalonFX TalonFXLfollow = new WPI_TalonFX(Drive.LFID);  
    private final WPI_TalonFX TalonFXR = new WPI_TalonFX(Drive.RID); 
    private final WPI_TalonFX TalonFXRfollow = new WPI_TalonFX(Drive.RFID);
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Drive.trackWidthMeters); 
    private final DifferentialDriveOdometry odometry;
    private final PigeonIMU pigeon;

    public DriveSubsystem(PigeonIMU pigeon) {
        this.pigeon = pigeon;
        configureMotor(TalonFXL);
        configureMotor(TalonFXR);
        configureMotor(TalonFXLfollow);
        configureMotor(TalonFXRfollow);
        TalonFXR.setInverted(true);
        TalonFXRfollow.setInverted(true);
        TalonFXLfollow.follow(TalonFXL);
        TalonFXRfollow.follow(TalonFXR);
        calibrate();
        odometry = new DifferentialDriveOdometry(new Rotation2d(getHeading()), 0, 0);
    }

    private void configureMotor(TalonFX talon) {
        talon.configFactoryDefault();
        talon.selectProfileSlot(0, 0);
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Drive.timeoutMs);
        talon.config_kP(0, Drive.kP);
        talon.config_kI(0, Drive.kI);
        talon.config_kD(0, Drive.kD);
        talon.config_kF(0, Drive.kF);
        talon.configNeutralDeadband(0.001);
        talon.configClosedLoopPeakOutput(0, Drive.maxDriveSpeed);
    }

    @Override
    public void periodic() {
    }

    public void setPower(double leftPower, double rightPower){
        TalonFXL.set(TalonFXControlMode.PercentOutput, leftPower);
        TalonFXR.set(TalonFXControlMode.PercentOutput, rightPower);
    }

    public void driveDistance(double distanceInches) {
        double sensorUnits = Encoder.fromDistance(distanceInches, Drive.encoderUnits, Drive.gearboxRatio, Drive.diameterInches);
        TalonFXL.set(TalonFXControlMode.Position, TalonFXL.getSelectedSensorPosition()+sensorUnits);
        TalonFXR.set(TalonFXControlMode.Position, TalonFXR.getSelectedSensorPosition()+sensorUnits);
    }

    public ControlMode getMode() {
        return TalonFXR.getControlMode();
    }
    
    public double getRightPostition() {
        return Encoder.toDistance(TalonFXR.getSelectedSensorPosition(), Drive.encoderUnits, Drive.gearboxRatio, Drive.diameterInches); 
        
    }
    
    public double getleftPostition() {
        return Encoder.toDistance(TalonFXL.getSelectedSensorPosition(), Drive.encoderUnits, Drive.gearboxRatio, Drive.diameterInches); 
    }

    public void calibrate() {
        TalonFXL.setSelectedSensorPosition(0);
        TalonFXLfollow.setSelectedSensorPosition(0);
        TalonFXR.setSelectedSensorPosition(0);
        TalonFXRfollow.setSelectedSensorPosition(0);
    }

    public void brake() {
        TalonFXL.setNeutralMode(NeutralMode.Brake);
        TalonFXLfollow.setNeutralMode(NeutralMode.Brake);
        TalonFXR.setNeutralMode(NeutralMode.Brake);
        TalonFXRfollow.setNeutralMode(NeutralMode.Brake);
    }

    public void setVoltage(double voltsL, double voltsR) {
        TalonFXL.setVoltage(voltsL);
        TalonFXR.setVoltage(voltsR);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
    }
    
    /**
     * Sets the velocity of the motors.
     * @param velocityL The velocity of the left motors in inches/second
     * @param velocityR The velocity of the right motors in inches/second
     */
    public void setVelocity(double velocityL, double velocityR) {
        TalonFXL.set(ControlMode.Velocity, fromVelocity(velocityL));
        TalonFXR.set(ControlMode.Velocity, fromVelocity(velocityR));
    }

    /**
     * Sets the velocity of the motor in encoder units velocity
     * @param leftVelocity Velocity of the left motor in encoder units per 100 ms
     * @param rightVelocity Velocity of the right motor in encoder units per 100 ms
     */
    public void setEncoderVelocity(double leftVelocity, double rightVelocity) {
        TalonFXL.set(TalonFXControlMode.Velocity, leftVelocity);
        TalonFXR.set(TalonFXControlMode.Velocity, rightVelocity);
    }

    public void setVelocity(DifferentialDriveWheelSpeeds speeds, double deadband) {
        if(Util.inRange(speeds.rightMetersPerSecond, deadband)) {
            TalonFXR.set(ControlMode.PercentOutput, 0);
        } else {
            TalonFXR.set(ControlMode.Velocity, fromVelocity(speeds.rightMetersPerSecond));
        }
        if(Util.inRange(speeds.leftMetersPerSecond, deadband)) {
            TalonFXL.set(ControlMode.PercentOutput, 0);
        } else {
            TalonFXL.set(ControlMode.Velocity, fromVelocity(speeds.rightMetersPerSecond));
        }
    }
    
    public double toVelocity(double velocity) {
        return Encoder.toVelocity(velocity, Drive.encoderUnits, Drive.gearboxRatio, Drive.diameterInches);
    }

    public double fromVelocity(double velocity) {
        return Encoder.fromVelocity(velocity, Drive.encoderUnits, Drive.gearboxRatio, Drive.diameterInches);
    }

    public double getVelocityRight() {
        return toVelocity(TalonFXR.getSelectedSensorVelocity());
    }

    public double getVelocityLeft() {
        return toVelocity(TalonFXL.getSelectedSensorVelocity());
    }

    public double getVelocity() {
        return kinematics.toChassisSpeeds(getWheelSpeeds()).vxMetersPerSecond;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getHeading() {
        return Util.boundedAngleDegrees(pigeon.getFusedHeading());
    }

}
