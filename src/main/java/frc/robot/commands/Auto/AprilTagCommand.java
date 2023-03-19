package frc.robot.commands.Auto;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Driving.MoveRelativeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class AprilTagCommand extends InstantCommand {
    
    // private final TrajectoryConfig config = new TrajectoryConfig(2, 1);
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final PigeonIMU pigeon;
    private Supplier<Position> position;    

    public AprilTagCommand(Supplier<Position> position, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PigeonIMU pigeon) {
        this.position = position;
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.pigeon = pigeon;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        double xOffset = 0;
        double yOffset = 0.3556;
        Pose2d targetPose = visionSubsystem.getPoseToTarget(Target.APRILTAG, pigeon);
        switch (position.get()) {
            case LEFT:
            xOffset = 0.47625; 
            break;
            case CENTER:
            xOffset = 0;
            break;
            case RIGHT: 
            xOffset = -0.47625;
            break;
        }
        Pose2d pose = new Pose2d((targetPose.getX() + xOffset), (targetPose.getY() + yOffset), (targetPose.getRotation().times(-1)));
        Command command = new MoveRelativeCommand(pose.getX(), pose.getY(), pose.getRotation().getDegrees(), driveSubsystem, pigeon);
        // Command command = getTrajectoryCommand(targetPose);
        command.schedule();
    }

    // private Command getTrajectoryCommand(Pose2d pose) {
    //     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), new ArrayList<>(), pose, config);
    //     return FollowTrajectory.getCommand(driveSubsystem, trajectory, driveSubsystem.getPose());
    // }

    public enum Position {
        LEFT,
        CENTER,
        RIGHT
    }


}
