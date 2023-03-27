package frc.robot.lib.motion;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;


/**
 * Keeps the robot on a fixed trajectory
 */
public class FollowTrajectory {

    private static NetworkTable kDebug = NetworkTableInstance.getDefault().getTable("ramsete");
    private static NetworkTableEntry kLeftReference = kDebug.getEntry("left_reference");
    private static NetworkTableEntry kRightReference = kDebug.getEntry("right_reference");

    private static SimpleMotorFeedforward kFeedforward;
    private static DifferentialDriveKinematics kKinematics;
    private static RamseteController kController;
    private static RamseteController kDisabledController;
    private static PIDController kLeftPidController;
    private static PIDController kRightPidController;

    private static double kTurnCompensation;

    
    /**
     * @param kS The kS constant(Feedforward)
     * @param kV The kV constant(Feedforward)
     * @param kA The kA constant(Feedforward)
     * @param b The B constant(RAMSETE)
     * @param zeta The Zeta constant(RAMSETE)
     * @param trackWidth The width of the drivetrain(Kinematics)
     * @param pidController The PID Controller to use
     * @param turnCompensation How much to overcorrect for turning aka how much the tracks are couple, 0 for a perfect drive
     */
    public static void config(double kS, double kV, double kA, double b, double zeta, double trackWidth, PIDController pidController, double turnCompensation) {
        kFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
        kKinematics = new DifferentialDriveKinematics(trackWidth);
        kController = new RamseteController(b, zeta);
        kLeftPidController = new PIDController(pidController.getP(), pidController.getI(), pidController.getD());
        kRightPidController = new PIDController(pidController.getP(), pidController.getI(), pidController.getD());
        kDisabledController = new RamseteController() {
            @Override
            public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                    double angularVelocityRefRadiansPerSecond) {
                return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
            }
        };
        kTurnCompensation = turnCompensation;
    }

    /**
     * Returns a RamseteCommand that follows the specified trajectory using no feedback
     * @param driveSubsystem The DriveSubsystem to use
     * @param trajectory The Trajectory to follow
     * @param zeroPose The position to start relative to
     * @return Returns a RamseteCommand that will follow the specified trajectory with the specified driveSubsystem
     */
    public static Command getCommandFeedforward(DriveSubsystem driveSubsystem, Trajectory trajectory, Pose2d zeroPose) {
        trajectory = trajectory.relativeTo(zeroPose);
        PIDController leftController = new PIDController(0, 0, 0);
        PIDController rightController = new PIDController(0, 0, 0);
        return new RamseteCommand(
                trajectory,
                driveSubsystem::getPose,
                kDisabledController,
                kFeedforward,
                kKinematics,
                driveSubsystem::getWheelSpeeds,
                leftController,
                rightController,
                (voltsL, voltsR) -> {
                    double turnCompensation = voltsR - voltsL;
                    turnCompensation *= kTurnCompensation;
                    driveSubsystem.setVoltage(voltsL - turnCompensation, voltsR + turnCompensation);

                    kLeftReference.setNumber(leftController.getSetpoint());
                    kRightReference.setNumber(rightController.getSetpoint());
                },
                driveSubsystem);
    }

    /**
     * Returns a RamseteCommand that follows the specified trajectory
     * @param driveSubsystem The DriveSubsystem to use
     * @param trajectory The Trajectory to follow
     * @param zeroPose The position to start relative to
     * @return Returns a RamseteCommand that will follow the specified trajectory with the specified driveSubsystem
     */
    public static Command getCommand(DriveSubsystem driveSubsystem, Trajectory trajectory, Pose2d zeroPose) {
        trajectory = trajectory.relativeTo(zeroPose);
        return new RamseteCommand(
                trajectory,
                driveSubsystem::getPose,
                kController,
                kFeedforward,
                kKinematics,
                driveSubsystem::getWheelSpeeds,
                kLeftPidController,
                kRightPidController,
                (voltsL, voltsR) -> {
                    double turnCompensation = voltsR - voltsL;
                    turnCompensation *= kTurnCompensation;
                    driveSubsystem.setVoltage(voltsL - turnCompensation, voltsR + turnCompensation);

                    kLeftReference.setNumber(kLeftPidController.getSetpoint());
                    kRightReference.setNumber(kRightPidController.getSetpoint());
                },
                driveSubsystem);
    }

    /**
     * Returns a RamseteCommand that follows the specified trajectory and uses the PID loop on the Talons
     * @param driveSubsystem The DriveSubsystem to use
     * @param trajectory The Trajectory to follow
     * @param zeroPose The position to start relative to
     * @return Returns a RamseteCommand that will follow the specified trajectory with the specified driveSubsystem
     */
    public static Command getCommandTalon(DriveSubsystem driveSubsystem, Trajectory trajectory, Pose2d zeroPose) {
        trajectory = trajectory.transformBy(new Transform2d(new Pose2d(), zeroPose));
        return new RamseteCommand(
                trajectory,
                driveSubsystem::getPose,
                kController,
                kKinematics,
                (velocityL, velocityR) -> {
                    driveSubsystem.setVelocity(velocityL, velocityR);

                    kLeftReference.setNumber(velocityL);
                    kRightReference.setNumber(velocityR);
                },
                driveSubsystem);
    }
    
    /**
     * Returns a RamseteCommand that follows a generated trajectory using no feedback
     * @param driveSubsystem The DriveSubsystem to use
     * @param start The position to start at
     * @param end The position to end at
     * @param maxVelocity The maximum velocity of the robot
     * @param maxAcceleration The maximum acceleration of the robot
     * @param maxVoltage The maximum voltage that can be applied to the motors
     * @param maxCentripetalAccleration The maximum centripetal acceleration of the robot
     * @param reversed If the trajectory should be reversed
     * @return Returns a RamseteCommand that will follow the specified trajectory with the specified driveSubsystem
     */
    public static Command getCommandFeedforward(DriveSubsystem driveSubsystem, Pose2d start, Pose2d end, double maxVelocity, double maxAcceleration, double maxVoltage, double maxCentripetalAccleration, boolean reversed) {
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.setReversed(reversed);
        config.addConstraint(new DifferentialDriveKinematicsConstraint(kKinematics, maxVelocity));
        config.addConstraint(new DifferentialDriveVoltageConstraint(kFeedforward, kKinematics, 6.0));
        config.addConstraint(new CentripetalAccelerationConstraint(10));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, new ArrayList<Translation2d>(), end,config);
        trajectory = trajectory.relativeTo(trajectory.getInitialPose());
        return getCommandFeedforward(driveSubsystem, trajectory, trajectory.getInitialPose());
    }

    /**
     * Returns a RamseteCommand that follows a generated trajectory
     * @param driveSubsystem The DriveSubsystem to use
     * @param start The position to start at
     * @param end The position to end at
     * @param maxVelocity The maximum velocity of the robot
     * @param maxAcceleration The maximum acceleration of the robot
     * @param maxVoltage The maximum voltage that can be applied to the motors
     * @param maxCentripetalAccleration The maximum centripetal acceleration of the robot
     * @param reversed If the trajectory should be reversed
     * @return Returns a RamseteCommand that will follow the specified trajectory with the specified driveSubsystem
     */
    public static Command getCommand(DriveSubsystem driveSubsystem, Pose2d start, Pose2d end, double maxVelocity, double maxAcceleration, double maxVoltage, double maxCentripetalAccleration, boolean reversed) {
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.setReversed(reversed);
        config.addConstraint(new DifferentialDriveKinematicsConstraint(kKinematics, maxVelocity));
        config.addConstraint(new DifferentialDriveVoltageConstraint(kFeedforward, kKinematics, 6.0));
        config.addConstraint(new CentripetalAccelerationConstraint(10));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, new ArrayList<Translation2d>(), end,config);
        trajectory = trajectory.relativeTo(trajectory.getInitialPose());
        return getCommand(driveSubsystem, trajectory, trajectory.getInitialPose());
    }

    /**
     * Returns a RamseteCommand that follows a generated trajectory and uses the PID loop on the Talons
     * @param driveSubsystem The DriveSubsystem to use
     * @param start The position to start at
     * @param end The position to end at
     * @param maxVelocity The maximum velocity of the robot
     * @param maxAcceleration The maximum acceleration of the robot
     * @param maxVoltage The maximum voltage that can be applied to the motors
     * @param maxCentripetalAccleration The maximum centripetal acceleration of the robot
     * @param reversed If the trajectory should be reversed
     * @return Returns a RamseteCommand that will follow the specified trajectory with the specified driveSubsystem
     */
    public static Command getCommandTalon(DriveSubsystem driveSubsystem, Pose2d start, Pose2d end, double maxVelocity, double maxAcceleration, double maxVoltage, double maxCentripetalAccleration, boolean reversed) {
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.setReversed(reversed);
        config.addConstraint(new DifferentialDriveKinematicsConstraint(kKinematics, maxVelocity));
        config.addConstraint(new DifferentialDriveVoltageConstraint(kFeedforward, kKinematics, 6.0));
        config.addConstraint(new CentripetalAccelerationConstraint(10));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, new ArrayList<Translation2d>(), end,config);
        trajectory = trajectory.relativeTo(trajectory.getInitialPose());
        return getCommandTalon(driveSubsystem, trajectory, trajectory.getInitialPose());
    }
} 