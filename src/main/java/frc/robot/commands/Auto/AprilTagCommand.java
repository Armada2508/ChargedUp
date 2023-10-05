package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Vision;
import frc.robot.lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class AprilTagCommand extends InstantCommand {
    
    private double zOffset = 1;
    private final double minZ = 0.05;
    private final double minX = 0.05;
    private final Supplier<Position> position;    
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    /**
     * 
     * @param position - side to go on relative to the tag
     * @param zOffset - distance away from the tag on the z axis (forwards, backwards) in meters
     * @param driveSubsystem
     * @param visionSubsystem
     */
    public AprilTagCommand(Supplier<Position> position, double zOffset, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.position = position;
        this.zOffset = zOffset;
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        visionSubsystem.setPipeline(Target.APRILTAG);
        double xOffset = 0;
        if (!visionSubsystem.hasTarget(Target.APRILTAG)) {
            cancel();
            return;
        }
        // Robot Frame
        Pose3d tagPose = visionSubsystem.getTargetPose();
        System.out.println("TagPose: " + tagPose);
        xOffset = switch (position.get()) {
            case LEFT -> xOffset = -0.549275; 
            case CENTER -> xOffset = 0;
            case RIGHT -> xOffset = 0.549275;
        };
        zOffset += Vision.centerToFront;
        // Translation Offset rotated into the robot's frame to be added onto target position
        Vector<N3> rvec = visionSubsystem.getRotationalVector();
        // Robot Frame
        Translation3d tagOffset = new Translation3d(xOffset, 0, zOffset).rotateBy(new Rotation3d(rvec));
        System.out.println("Tag Offset: " + tagOffset);
        double skew = visionSubsystem.getSkew();
        // Odometry Frame
        double x = tagPose.getZ() + tagOffset.getZ();
        double y = -(tagPose.getX() + tagOffset.getX());
        if (Math.abs(x) < minZ && Math.abs(y) < minX) {
            System.out.println("\nToo Close!\n");
            cancel();
            return;
        }
        Pose2d targetPose = new Pose2d(x, y, new Rotation2d(-skew));
        Command command = getTrajectoryCommand(targetPose, new ArrayList<>());
        command.schedule();
    }

    private Command getTrajectoryCommand(Pose2d pose, List<Translation2d> intermediatePoses) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), intermediatePoses, pose, Drive.trajectoryConfig);
        System.out.println("Starting Odometry: " + driveSubsystem.getPose());
        System.out.println("Trajectory: " + trajectory.transformBy(new Transform2d(new Pose2d(), driveSubsystem.getPose())) + ", \nFrom: " + new Pose2d() + ", \nTo: " + pose);
        return FollowTrajectory.getCommandTalon(driveSubsystem, trajectory, driveSubsystem.getPose());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public enum Position {
        LEFT,
        CENTER,
        RIGHT
    }

}
