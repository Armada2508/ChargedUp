package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.Lib.util.Util;

public class Constants {

    public static final int pigeonID = 8;
    public static final int timeoutMs = 30;
    
    public static final class Drive {
        public static final int RID = 0;
        public static final int RFID = 1;
        public static final int LID = 2;
        public static final int LFID = 3;
        public static final int turnAdjustment = 3;
        public static final double speedMultiplier = 1; // For Fredy
        public static final double joystickDeadband = 0.05;
        public static final double slewRate = 1; // This is what helps you not stop abruptly, higher value = stop faster

        // Closed Loop Driving
        public static final double kP = 0.1; 
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0; // Probably keep this at 0
        public static final double maxDriveSpeed = 1;

        // Closed Loop Turning (WPILib not on Falcons)
        public static final double turnkP = 0.01;
        public static final double turnkI = 0;
        public static final double turnkD = 0;
        public static final double maxTurnSpeed = 0.25;

        // DriveBase
        public static final double wheelDiameterMeters = Units.inchesToMeters(6);
        public static final int encoderUnitsPerRev = 2048;
        public static final double gearboxRatio = 10.7;
        public static final double trackWidthMeters = Util.inchesToMeters(24.5);
    }

    public static final class Balance {
        // Roll PID
        public static final double rollkP = 0.05;
        public static final double rollkI = 0;
        public static final double rollkD = 0;
        // Offsets, gathered at start of match.
        public static double pitchOffset = 0;
        public static double rollOffset = -2;
        // Speeds
        public static final double maxSpeed = 0.15;
        public static final double pitchSpeed = 0.12;
        // Pitch angle to start driving at
        public static final double balanceAngle = 10;
    }

    public static final class Vision {
        // Distances
        public static final double distanceToBumperMeters = Units.inchesToMeters(11.5);
        // Target Heights
        public static final double coneHeightMeters = 0;
        public static final double cubeHeightMeters = Units.inchesToMeters(58);
        public static final double aprilTagHeightMeters = 0;
        // Camera
        public static final double cameraHeightMeters = Units.inchesToMeters(50);
        public static final double cameraPitchRadians = Units.degreesToRadians(0.1); 
    }

    public static final class Arm {
        public static final int motorID = 4;
        public static final int motorIDFollow = 5;
        public static final int encoderUnitsPerRev = 2048;
        public static final int gearboxRatio = 100;

        // Closed Loop
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double gravityFeedForward = 0.07;
        public static final double maxSpeed = 0.25;

        public static final int minDegrees = -18; // Straight down is 0.
        public static final int maxDegrees = 90;
        // IK
        public static final double jointLengthInches = 35.75;
    }

    public static final class Wrist {
        public static final int motorID = 6;
        public static final int encoderUnitsPerRev = 2048;
        public static final int driverPully = 20;
        public static final int drivenPulley = 20;
        public static final int pulleyRatio = drivenPulley/driverPully;

        // Closed Loop
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double gravityFeedForward = 0.07;
        public static final double maxSpeed = 0.25;

        public static final int minDegrees = -30; // Straight out is 0.
        public static final int maxDegrees = 30;

        // IK
        public static final double jointLengthInches = 13;
    }

    public static final class Gripper {
        public static final int motorID = 7;
        public static final int encoderUnitsPerRev = 2048;
        public static final int gearboxRatio = 100;

        // Closed Loop
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double maxSpeed = 0.25;

        // Bad Engineering Constants
        public static final double armSensorOffset = 3;
        public static final double wristSensorOffset = 1;

        public static final double min = 0;
        public static final double max = 1;
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
