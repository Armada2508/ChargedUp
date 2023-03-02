package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Lib.motion.FollowTrajectory;
import frc.robot.commands.Driving.MoveRelativeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class AprilTagCommand extends InstantCommand {
    
    private final TrajectoryConfig config = new TrajectoryConfig(2, 1);
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
        private double xOffset;
        private double yOffset = 0.3556;
        Pose2d targetPose = visionSubsystem.getPoseToTarget(Target.APRILTAG);
        // Pose2d targetPose = new Pose2d(5, 5, Rotation2d.fromDegrees(0));
        switch (position) {
            case LEFT:
            xOffset = 0.47625; //temporary
            break;
            case CENTER:
            xOffset = 0;
            break;
            case RIGHT: 
            xOffset = -0.47625; //temporary
            break;
        }
        Pose2d pose = new Pose2d((targetPose.getX() + xOffset), (targetPose.getY() + yOffset), Rotation2d.fromDegrees(0));
        Command command = new MoveRelativeCommand(targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees(), driveSubsystem, pigeon);
        command.schedule();
        getTrajectoryCommand(targetPose);
    }

    private Command getTrajectoryCommand(Pose2d pose) {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), new ArrayList<>(), pose, config);
        return FollowTrajectory.getCommand(driveSubsystem, trajectory, driveSubsystem.getPose());
    }

    public enum Position {
        LEFT,
        CENTER,
        RIGHT
    }

    // private List<Trajectory.State> getSamples(Trajectory trajectory) {
    //     double time = trajectory.getTotalTimeSeconds() / samples;
    //     List<Trajectory.State> states = new ArrayList<>();
    //     for (int i = 0; i < samples; i++) {
    //         states.add(trajectory.sample(time * i));
    //     }
    //     return states;
    // }

    //~ take in supplier from position
    //~ in initialize, make a case/switch for all 3 positions
    //~ depending on the case, grab certain x's and y's
    //~ define the x's and y's early on
    //~ center x is 0


}
