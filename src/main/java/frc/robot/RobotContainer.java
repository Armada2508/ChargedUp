package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.LineUpCommand;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.commands.Driving.AutoTurnCommand;
import frc.robot.commands.Driving.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class RobotContainer {

    private final Joystick joystick = new Joystick(0);
    private final VisionSubsystem vision = new VisionSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final DriveSubsystem driveSubsystem;
    private final PigeonIMU pigeon;
    private Thread visionThread;
    // ! Auto Turn Command Drift
    RobotContainer(PigeonIMU pigeon) {
        this.pigeon = pigeon;
        this.driveSubsystem = new DriveSubsystem(pigeon);
        driveSubsystem.setDefaultCommand(new DriveCommand(() -> joystick.getRawAxis(1)*-1, () -> joystick.getRawAxis(0)*-1, driveSubsystem)); // default to driving from joystick input
        configureCamera();
        configureShuffleboard();
        configureButtons();
        // new CalibrateArmCommand(armSubsystem).schedule();
    }

    private void configureButtons() {
        new JoystickButton(joystick, 12).onTrue(new InstantCommand(() -> {
            Command command = driveSubsystem.getCurrentCommand(); 
            if (command instanceof BalanceCommand) {
                command.cancel();
            }
        }));
        new JoystickButton(joystick, 11).onTrue(new BalanceCommand(driveSubsystem, pigeon));
        new JoystickButton(joystick, 10).onTrue(new InstantCommand(vision::limelightOFF));
        new JoystickButton(joystick, 9).onTrue(new InstantCommand(vision::limelightON));
        new JoystickButton(joystick, 5).whileTrue(Commands.startEnd(() -> armSubsystem.setPower(0.1), armSubsystem::stop, armSubsystem));
        new JoystickButton(joystick, 3).whileTrue(Commands.startEnd(() -> armSubsystem.setPower(-0.1), armSubsystem::stop, armSubsystem));
        new JoystickButton(joystick, 6).onTrue(new LineUpCommand(driveSubsystem, vision, Target.POLE));
        new JoystickButton(joystick, 4).onTrue(new LineUpCommand(driveSubsystem, vision, Target.CONE));
   }

    private void configureShuffleboard() {
        // Pigeon
        Shuffleboard.getTab("Pigeon")
            .addDouble("Pigeon Yaw", () -> pigeon.getYaw())
            .withPosition(0, 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -720, "max", 720))
            .withSize(4, 3);
        Shuffleboard.getTab("Pigeon")
            .addDouble("Pigeon Pitch", () -> pigeon.getPitch())
            .withPosition(4, 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -90, "max", 90))
            .withSize(4, 3);
        Shuffleboard.getTab("Pigeon")
            .addDouble("Pigeon Roll", () -> pigeon.getRoll())
            .withPosition(8, 0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", -90, "max", 90))
            .withSize(4, 3);

        // Vision
        Shuffleboard.getTab("Vision")
            .addBoolean("Target Found", vision::hasTarget)
            .withPosition(0, 0)
            .withSize(4, 3);
        Shuffleboard.getTab("Vision")
            .addNumber("Target X", vision::getTargetX)
            .withPosition(4, 0)
            .withSize(4, 3)
            .withProperties(Map.of("min", -29.8, "max", 29.8))
            .withWidget(BuiltInWidgets.kNumberBar);
        Shuffleboard.getTab("Vision")
            .addNumber("Target Y", vision::getTargetY)
            .withPosition(8, 0)
            .withSize(4, 3)
            .withProperties(Map.of("min", -24.85, "max", 24.85))
            .withWidget(BuiltInWidgets.kNumberBar);
        Shuffleboard.getTab("Vision")
            .addNumber("Target Area", vision::getTargetArea)
            .withPosition(12, 0)
            .withSize(4, 3)
            .withProperties(Map.of("min", 0, "max", 100))
            .withWidget(BuiltInWidgets.kNumberBar);
        Shuffleboard.getTab("Vision")
            .add(CameraServer.getVideo().getSource())
            .withPosition(0, 4)
            .withSize(4, 3)
            .withWidget(BuiltInWidgets.kCameraStream);
        Shuffleboard.getTab("Vision")
            .add(CameraServer.getVideo("OpenCV").getSource())
            .withPosition(9, 4)
            .withSize(4, 3)
            .withWidget(BuiltInWidgets.kCameraStream);
        Shuffleboard.selectTab("Vision");
    }

    private void configureCamera() {
        UsbCamera camera = CameraServer.startAutomaticCapture("Camera", 0);
        camera.setExposureManual(25);
        camera.setFPS(12);
        camera.setResolution(426, 240);
        visionThread = new Thread(() -> {
            CvSink sink = CameraServer.getVideo();
            CvSource source = CameraServer.putVideo("OpenCV", 426, 260);
            Mat mat = new Mat();
            Scalar lower = new Scalar(0, 50, 50);
            Scalar upper = new Scalar(50, 255, 255);
            while(!visionThread.isInterrupted()) {
                if (sink.grabFrame(mat) == 0) {
                    source.notifyError(sink.getError());
                    continue;
                }
                Imgproc.blur(mat, mat, new Size(10, 10));
                Core.inRange(mat, lower, upper, mat);
                List<MatOfPoint> contours = new ArrayList<>();
                Imgproc.findContours(mat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
                Imgproc.drawContours(mat, contours, -1, new Scalar(0, 255, 0), 2);
                source.putFrame(mat);
            }
        });
        visionThread.start();
    }

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            new AutoTurnCommand(driveSubsystem, pigeon, 90),
            new AutoDriveCommand(24, driveSubsystem),
            new AutoTurnCommand(driveSubsystem, pigeon, -90)
            // new AutoDriveCommand(48, driveSubsystem)
        );
    }

}
