package frc.robot.commands.Auto;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Lib.util.Util;
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
        visionSubsystem.setPipeline(Target.APRILTAG);
        double xOffset = 0;
        double zOffset = 0.3556;
        if (!visionSubsystem.hasTarget(Target.APRILTAG)) {
            cancel();
            return;
        }
        Pose2d cameraPose = visionSubsystem.getPoseToTarget(Target.APRILTAG, pigeon);
        System.out.println("Camera Pose: " + cameraPose);
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
        xOffset = 0;
        zOffset = 1;
        double[] rvecArray = visionSubsystem.getRotationalVector(Target.APRILTAG);
        Vector<N3> rvec = VecBuilder.fill(rvecArray[0], rvecArray[1], rvecArray[2]); 
        // Translation Offset rotated into the camera's frame to be added onto target position
        Translation3d tagOffset = new Translation3d(xOffset, 0, zOffset).rotateBy(new Rotation3d(rvec));
        // Used to find april tag skew angle
        Translation3d tagNormal = new Translation3d(0, 0, 1).rotateBy(new Rotation3d(rvec));
        System.out.println("Tag Normal: " + tagNormal);
        // skew
        double finalAngleRad = Util.boundedAngle(Math.atan2(tagNormal.getX(), tagNormal.getZ()) + Math.PI);
        System.out.println("AprilTag Translation in Camera Frame: " + tagOffset);
        System.out.println("Final Translation: " + (cameraPose.getX() + tagOffset.getX()) + " " + (cameraPose.getY() + tagOffset.getZ()) + " " + finalAngleRad);
        Command command = new MoveRelativeCommand(cameraPose.getX() + tagOffset.getX(), cameraPose.getY() + tagOffset.getZ(), finalAngleRad, driveSubsystem, pigeon);
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
