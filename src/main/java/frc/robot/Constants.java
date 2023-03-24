package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.Util;

public class Constants {

    public static final int pigeonID = 8;
    public static final int timeoutMs = 30;
    
    public static final class Drive {
        public static final int RID = 0;
        public static final int RFID = 1;
        public static final int LID = 2;
        public static final int LFID = 3;
        public static final double speedAdjustment = 0.25; // For Fredy
        public static final double turnAdjustment = 0.5;
        public static final double trimAdjustment = 0.15;
        public static final double joystickDeadband = 0.07;
        public static final double slewRate = 1.5; // This is what helps you not stop abruptly, higher value = stop faster
        public static final double slowSpeed = 0.2;

        // Closed Loop Driving
        public static final double kP = 0.3; 
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0; // Probably keep this at 0
        public static final double maxOutput = 1;
        public static final double nominalOutputLeft = 0.04;
        public static final double nominalOutputRight = 0.03;

        // Closed Loop Turning (WPILib not on Falcons)
        public static final double turnkP = 0.015;
        public static final double turnkI = 0;
        public static final double turnkD = 0.001;
        public static final double maxTurnSpeed = 0.2;
        
        // DriveBase
        public static final double wheelDiameterMeters = Units.inchesToMeters(6);
        public static final int encoderUnitsPerRev = 2048;
        public static final double gearboxRatio = 10.71;
        public static final double trackWidthMeters = Util.inchesToMeters(24.5);
    }

    public static final class Balance {
        // Offsets, gathered at start of match.
        public static double pitchOffset = 0;
        // Speeds
        public static final double pitchSpeed = 0.15;
        // Pitch angle to start driving at
        public static final double stationAngle = 13;
        public static final double angleToStop = 10;
    }

    public static final class Vision {
        // Robot Frame
        public static final Translation3d cameraTranslationOffset = new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(13.5));
        public static final Rotation3d cameraRotationOffset = new Rotation3d(0, Units.degreesToRadians(142.65), 0);
        public static final Pose3d cameraPoseOffset = new Pose3d(cameraTranslationOffset, new Rotation3d());
        public static final Transform3d cameraToRobotTransform = new Transform3d(new Pose3d(), Vision.cameraPoseOffset);
        public static final double centerToFront = Units.inchesToMeters(14.5);
        // Target Heights
        public static final double coneHeightMeters = 0;
        public static final double cubeHeightMeters = 0;
        public static final double aprilTagHeightMeters = 0;
        // Camera
        public static final double cameraHeightMeters = Units.inchesToMeters(50.3);
        public static final double cameraPitchRadians = Units.degreesToRadians(0.1); 
    }

    public static final class Arm {
        public static final int motorID = 4;
        public static final int motorIDFollow = 5;
        public static final int encoderUnitsPerRev = 2048;
        public static final int gearboxRatio = 100;

        // Closed Loop
        // no kF, kF bad, is a circle, L kF
        public static final double kP = 0.2;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double gravityFeedForward = 0.035; 
        public static final double maxOutput = 1;
        public static final double minOutput = 0;
        public static final double maxVelocity = 90; // deg/sec

        public static final int minDegrees = -23; // Straight down is 0.
        public static final int maxDegrees = 105;
        public static final int limitMargin = 2;

        // IK
        public static final double jointLengthInches = 35.75;
    }

    public static final class Wrist {
        public static final int motorID = 7;
        public static final int encoderUnitsPerRev = 2048;
        public static final double gearboxRatio = 100;
        public static final double driverPully = 32;
        public static final double drivenPulley = 46;
        public static final double movementRatio = (drivenPulley/driverPully) * gearboxRatio;

        // Closed Loop
        public static final double kP = 0.15;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double maxOutput = 1;
        public static final double minOutput = 0;
        public static final double maxVelocity = 90; // deg/sec

        public static final int minDegrees = -90; // Parallel with the arm is 0.
        public static final int maxDegrees = 90;

        // IK
        public static final double jointLengthInches = 13;
    }

    public static final class Gripper {
        public static final int motorID = 6;
        public static final int encoderUnitsPerRev = 2048;
        public static final int gearboxRatio = 100;

        // Closed Loop
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double maxOutput = 1;
        public static final double minOutput = 0;
        public static final double maxVelocity = 1;

        // Bad Engineering Constants
        public static final double armSensorMultiplier = .75;
        public static final double wristSensorMultiplier = -.52;

        public static final double min = 0;
        public static final double max = 1;

        // Grab Constants
        public static final double almostClosed = max - 0.05;
        public static final double grabCone = max + 0.0;
        public static final double grabCube = max;
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
