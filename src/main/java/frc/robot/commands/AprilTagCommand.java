package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class AprilTagCommand {

    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public AprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    public void run(Pose2d poseOffset) {
        Pose2d targetPose = visionSubsystem.getPoseToTarget(Target.APRILTAG);
        targetPose = targetPose.plus(new Transform2d(new Pose2d(), poseOffset));
        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), null, targetPose, config);
        Command command = FollowTrajectory.getCommand(driveSubsystem, trajectory, driveSubsystem.getPose());
        command.schedule();
    }

    public void run() {
        run(new Pose2d());
    }

}
