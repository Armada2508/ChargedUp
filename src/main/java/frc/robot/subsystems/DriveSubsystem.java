package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.Lib.Encoder;
import frc.robot.Lib.util.Util;


public class DriveSubsystem extends SubsystemBase{

    private MotorControllerGroup right;
    private MotorControllerGroup left;
    private WPI_TalonFX TalonFXL; 
    private WPI_TalonFX TalonFXLfollow; 
    private WPI_TalonFX TalonFXR; 
    private WPI_TalonFX TalonFXRfollow;
    private final PIDController pidController = new PIDController(0.125, 0, 0);
    private final DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(Drive.kTrackWidth); 
    private final DifferentialDriveOdometry odometry = null; // fix
    private final PigeonIMU mImu = new PigeonIMU(0);

    public DriveSubsystem() {
        TalonFXL = new WPI_TalonFX(Drive.LID); 
        TalonFXLfollow = new WPI_TalonFX(Drive.LFID); 
        TalonFXR = new WPI_TalonFX(Drive.RID); 
        TalonFXRfollow = new WPI_TalonFX(Drive.RFID);
        configureMotor(TalonFXL);
        configureMotor(TalonFXR);
        TalonFXL.setInverted(true);
        TalonFXLfollow.setInverted(true);
        TalonFXLfollow.follow(TalonFXL);
        TalonFXRfollow.follow(TalonFXR);
        left =  new MotorControllerGroup(TalonFXL, TalonFXLfollow);
        right = new MotorControllerGroup(TalonFXR, TalonFXRfollow);
    }

    @Override
    public void periodic() {}

    public void setPower(double leftPower, double rightPower){
        left.set(leftPower);
        right.set(rightPower);
    }

    public void driveDistance(double distanceInches) {
        double sensorUnits = Encoder.fromDistance(distanceInches, Drive.encoderUnits, Drive.gearboxRatio, Drive.diameter);
        TalonFXL.set(TalonFXControlMode.Position, TalonFXL.getSelectedSensorPosition()+sensorUnits);
        TalonFXR.set(TalonFXControlMode.Position, TalonFXR.getSelectedSensorPosition()+sensorUnits);
    }

    public ControlMode getMode() {
        return TalonFXR.getControlMode();
    }
    
    public double getRightPostition() {
        return Encoder.toDistance(TalonFXR.getSelectedSensorPosition(), Drive.encoderUnits, Drive.gearboxRatio, Drive.diameter); 
        
    }
    
    public double getleftPostition() {
        return Encoder.toDistance(TalonFXL.getSelectedSensorPosition(), Drive.encoderUnits, Drive.gearboxRatio, Drive.diameter); 
    }

    private void configureMotor(WPI_TalonFX motor) {
        motor.config_kP(0, pidController.getP());
        motor.config_kI(0, pidController.getI());
        motor.config_kD(0, pidController.getD());
    }
    
    public void callibrate() {
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

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
    }

    public void setVelocity(DifferentialDriveWheelSpeeds speeds) {
        TalonFXR.set(ControlMode.Velocity, fromVelocity(speeds.rightMetersPerSecond));
        TalonFXL.set(ControlMode.Velocity, fromVelocity(speeds.leftMetersPerSecond));
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

    public void setVoltage(double voltsL, double voltsR) {
        TalonFXL.setVoltage(voltsL);
        TalonFXR.setVoltage(voltsR);
    }

    //Maybe change bc it's only checking one motor on the right
    public double getVelocityRight() {
        return toVelocity((int)TalonFXR.getSelectedSensorVelocity());
    }

    //Maybe change bc it's only checking one motor on the left
    public double getVelocityLeft() {
        return toVelocity((int)TalonFXL.getSelectedSensorVelocity());
    }

    public double getVelocity() {
        return mKinematics.toChassisSpeeds(getWheelSpeeds()).vxMetersPerSecond;
    }

    public double toVelocity(int velocity) {
        return Encoder.toVelocity(velocity, Drive.kFeedbackConfig.getEpr(), Drive.kFeedbackConfig.getGearRatio(), Drive.diameter);
    }

    public double fromVelocity(double velocity) {
        return Encoder.fromVelocity(velocity, Drive.kFeedbackConfig.getEpr(), Drive.kFeedbackConfig.getGearRatio(), Drive.diameter);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getHeading() {
        return Util.boundedAngleDegrees(mImu.getFusedHeading());
    }

}
