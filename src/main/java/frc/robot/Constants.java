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
        public static final double kP = 0.25; 
        public static final double kI = 0;
        public static final double kD = 0;

        // Turning PID
        public static final double turnkP = 0.02;
        public static final double turnkD = 0.005;

        public static final double minSpeed = 0.18;
        
        public static final double diameterInches = 6;
        public static final int encoderUnits = 4096;
        public static final double gearboxRatio = 12.75;
        public static final double trackWidthMeters = Util.inchesToMeters(23.5);

        public static final FeedbackConfig feedbackConfig = new FeedbackConfig(FeedbackDevice.IntegratedSensor, encoderUnits, gearboxRatio);
    }

    public static final int pigeonID = 8;

    public static final class Vision {
        public static final String cameraName = "Camera";
        public static final double distanceBetweenPolesInches = 17;
        public static final double distanceHighPoleToFrontInches = 39.75;
        public static final double distanceToBumperInches = 12.5;
        // Target Heights
        public static final double highPoleHeightInches = 43.84375;
        public static final double midPoleHeightInches = 24.0625; 
        public static final double coneHeightInches = 0;
        public static final double cubeHeightInches = 4.75;
        // Camera
        public static final double cameraHeightInches = 50;
        public static final double cameraAngleMountedDegrees = 0.5; 
        // Camera Pipelines
        public static final int coneLandscapePipeline = 0;
        public static final int cubePipeline = 1;
        public static final int conePortraitPipeline = 2;
    }

    public static final class Arm {
        public static final int motorID = 4;
        public static final int motorIDFollow = 5;
        public static final int encoderUnits = 2048;
        public static final int gearboxRatio = 100;
        public static final double degreesPerEncoderUnit = (360/encoderUnits)/gearboxRatio;

        public static final int minDegrees = -18;
        public static final int maxDegrees = 90;
        public static final double gravityFeedForward = 0.07;
    }

    public static final class Wrist {
        public static final int motorID = 7;
        public static final int encoderUnits = 2048;
        public static final int gearboxRatio = 100;
        public static final double degreesPerEncoderUnit = (360/encoderUnits)/gearboxRatio;

        public static final int minDegrees = -30;
        public static final int maxDegrees = 30;
        public static final double gravityFeedForward = 0.07;
    }

    public static final class Gripper {
        public static final int motorID = 6;
        public static final int encoderUnits = 2048;
        public static final int gearboxRatio = 100;
        public static final double degreesPerEncoderUnit = (360/encoderUnits)/gearboxRatio;
    }

    public static final class Balance {
        // PID
        public static final double pitchkP = 0.015;
        public static final double pitchkI = 0;
        public static final double pitchkD = 0.003;
        public static final double rollkP = 0.03;
        public static final double rollkI = 0;
        public static final double rollkD = 0;
        // Offsets
        public static final double rollOffset = -2;
        // Speeds
        public static final double maxSpeed = 0.4;
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
