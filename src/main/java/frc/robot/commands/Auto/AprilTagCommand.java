package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Vision;
import frc.robot.lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class AprilTagCommand extends InstantCommand {
    
    private final TrajectoryConfig config = new TrajectoryConfig(1.5, 0.5);
    private final Supplier<Position> position;    
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public AprilTagCommand(Supplier<Position> position, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.position = position;
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        visionSubsystem.setPipeline(Target.APRILTAG);
        double xOffset = 0;
        double zOffset = 0.3556;
        if (!visionSubsystem.hasTarget(Target.APRILTAG)) {
            cancel();
            return;
        }
        Pose3d tagPose = visionSubsystem.getTargetPose();
        xOffset = switch (position.get()) {
            case LEFT -> xOffset = 0.47625; 
            case CENTER -> xOffset = 0;
            case RIGHT -> xOffset = -0.47625;
        };
        xOffset = 0;
        zOffset = 1;
        zOffset += Vision.centerToFront;
        // Translation Offset rotated into the camera's frame to be added onto target position
        Vector<N3> rvec = visionSubsystem.getRotationalVector();
        Translation3d tagOffset = new Translation3d(xOffset, 0, zOffset).rotateBy(new Rotation3d(rvec));
        double skew = visionSubsystem.getSkew();
        Pose2d targetPose = new Pose2d(tagPose.getX() + tagOffset.getX(), tagPose.getZ() + tagOffset.getZ(), new Rotation2d(skew));
        targetPose = new Pose2d(1, 0, new Rotation2d());
        // Command command = new MoveRelativeCommand(targetPose.getX(), targetPose.getY(), skew, driveSubsystem, pigeon);
        Command command = getTrajectoryCommand(targetPose);
        command.schedule();
    }

    private Command getTrajectoryCommand(Pose2d pose) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), new ArrayList<>(), pose, config);
        System.out.println("Trajectory: " + trajectory + ", \nFrom: " + new Pose2d() + ", \nTo: " + pose);
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
