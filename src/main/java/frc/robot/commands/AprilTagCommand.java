package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AprilTagCommand extends InstantCommand {
    
    private Pose2d offset = new Pose2d();
    private final int samples = 5;
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
        // Pose2d targetPose = visionSubsystem.getPoseToTarget(Target.APRILTAG);
        Pose2d targetPose = new Pose2d(5, 5, Rotation2d.fromDegrees(0));
        // targetPose = targetPose.plus(new Transform2d(new Pose2d(), offset));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), new ArrayList<>(), targetPose, config);
        List<Trajectory.State> samples = getSamples(trajectory);
        // Command command = FollowTrajectory.getCommand(driveSubsystem, trajectory, driveSubsystem.getPose());
        // command.schedule();
        System.out.println(samples);
        driveSubsystem.doMotionProfile(samples);
    }

    private List<Trajectory.State> getSamples(Trajectory trajectory) {
        double time = trajectory.getTotalTimeSeconds() / samples;
        List<Trajectory.State> states = new ArrayList<>();
        for (int i = 0; i < samples; i++) {
            states.add(trajectory.sample(time * i));
        }
        return states;
    }

}
