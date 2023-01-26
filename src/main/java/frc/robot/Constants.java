package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.Lib.config.FeedbackConfig;
import frc.robot.Lib.util.Util;

public class Constants {
    
    public static final class Drive {
        public static final int RID = 0;
        public static final int RFID = 1;
        public static final int LID = 2;
        public static final int LFID = 3;
        public static final int turnAdjustment = 3;

        // Driving PID
        public static final double kP = 0.01; 
        public static final double kI = 0;
        public static final double kD = 0.01;

        // Turning PID
        public static final double turnkP = 0.02;
        public static final double minSpeed = 0.18;
        
        public static final double diameter = 6;
        public static final int encoderUnits = 2048;
        public static final double gearboxRatio = 10.7;
        public static final double kTrackWidth = Util.inchesToMeters(23.5);

        public static final FeedbackConfig kFeedbackConfig = new FeedbackConfig(FeedbackDevice.IntegratedSensor, encoderUnits, gearboxRatio);
    }

    public static final int pigeonID = 8;

    public static final class Vision {
        // Target Heights
        public static final double highPoleHeight = 41.73; // inches
        public static final double midPoleHeight = 34.25; // inches
        // Target Desired Distances
        public static final int poleDistance = 23; // inches
        public static final int coneDistance = 6; // inches
        // Limelight
        public static final int cameraHeight = 19; // inches
        public static final int cameraAngle = 0; // degrees 
        // Limelight Pipelines
        public static final int reflectionPipeline = 0;
        public static final int colorPipeline = 1;
    }

    public static final class Arm {
        public static final int motorID = 4;
        public static final int motorIDFollow = 5;
        public static final int encoderUnits = 2048;
        public static final int gearboxRatio = 100;
        public static final double degreesPerEncoderUnit = (360/encoderUnits)/gearboxRatio;

        public static final int minDegrees = 0;
        public static final int maxDegrees = 90;
    }
    //! gearbox ratio not finalized
    public static final class Wrist {
        public static final int motorID = 7;
        public static final int encoderUnits = 2048;
        public static final int gearboxRatio = 100;
        public static final double degreesPerEncoderUnit = (360/encoderUnits)/gearboxRatio;

        public static final int minDegrees = -30;
        public static final int maxDegrees = 30;
    }
    //! gearbox ratio not finalized
    public static final class Gripper {
        public static final int motorID = 6;
        public static final int encoderUnits = 2048;
        public static final int gearboxRatio = 100;
        public static final double degreesPerEncoderUnit = (360/encoderUnits)/gearboxRatio;
    }

    // ========================================
    //    Global Motor Controller Constants
    // ========================================A
    public static final class MotorController {

        // Status frames are sent over CAN that contain data about the Talon.
        // They are broken up into different pieces of data and the frequency
        // at which they are sent can be changed according to your needs.
        // The period at which their are sent is measured in ms

        public static final int kTalonFrame1Period = 20;  // How often the Talon reports basic info(Limits, limit overrides, faults, control mode, invert)
        public static final int kTalonFrame2Period = 20;  // How often the Talon reports sensor info(Sensor position/velocity, current, sticky faults, profile)
        public static final int kTalonFrame3Period = 160;  // How often the Talon reports non selected quad info(Position/velocity, edges, quad a and b pin, index pin)
        public static final int kTalonFrame4Period = 160;  // How often the Talon reports additional info(Analog position/velocity, temperature, battery voltage, selected feedback sensor)
        public static final int kTalonFrame8Period = 160;  // How often the Talon reports more encoder info(Talon Idx pin, PulseWidthEncoded sensor velocity/position)
        public static final int kTalonFrame10Period = 160;  // How often the Talon reports info on motion magic(Target position, velocity, active trajectory point)
        public static final int kTalonFrame13Period = 160; // How often the Talon reports info on PID(Error, Integral, Derivative)

    }

}
