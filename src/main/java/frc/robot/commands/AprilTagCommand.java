package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Vision;
import frc.robot.Lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AprilTagCommand extends CommandBase {
    
    private final int[] redSet = {1, 2, 3};
    private final int[] blueSet = {6, 7, 8};
    private final double maxVelocity = 6;
    private final double maxAcceleration = 6;
    private final TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));
    private int[] currentSet = new int[0];
    private int currentTag = -1;
    private Pose2d currentPose = new Pose2d();
    private DriveSubsystem driveSubsystem;
    private PhotonSubsystem photonSubsystem = new PhotonSubsystem();
    private VisionSubsystem visionSubsystem;

    public AprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
    }
    
    @Override
    public void initialize() {
        currentTag = visionSubsystem.getTagID();
        Alliance team = DriverStation.getAlliance();
        currentSet = switch(team) {
            case Red -> {
                visionSubsystem.setPipeline(Vision.redPipeline);
                yield redSet;
            }
            case Blue -> {
                visionSubsystem.setPipeline(Vision.bluePipeline);
                yield blueSet;
            }
            case Invalid -> new int[0];
        };
        // double[] pose = visionSubsystem.getBotPose();
        if (photonSubsystem.hasTargets()) {
            Transform3d transform = photonSubsystem.getBestTarget().getBestCameraToTarget();
            currentPose = new Pose2d(transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
            System.out.println("Pose: " + currentPose);
            Translation2d targetTranslation = new Translation2d(
                currentPose.getTranslation().getX(), currentPose.getTranslation().getY()+2
            );
            Pose2d targetPose = new Pose2d(targetTranslation, currentPose.getRotation());
            Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                currentPose, new ArrayList<Translation2d>(), targetPose, config
            );
            FollowTrajectory.getCommandTalon(driveSubsystem, trajectory, currentPose);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
