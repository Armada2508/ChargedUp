package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class AprilTagCommand extends InstantCommand {
    
    private Pose2d offset = new Pose2d();
    private final TrajectoryConfig config = new TrajectoryConfig(1, 1);
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public AprilTagCommand(Pose2d offset, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.offset = offset;
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void initialize() {
        Pose2d targetPose = visionSubsystem.getPoseToTarget(Target.APRILTAG);
        targetPose = targetPose.plus(new Transform2d(new Pose2d(), offset));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), new ArrayList<>(), targetPose, config);
        Command command = FollowTrajectory.getCommand(driveSubsystem, trajectory, driveSubsystem.getPose());
        command.schedule();
    }

}
