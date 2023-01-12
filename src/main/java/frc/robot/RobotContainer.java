package frc.robot;

import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.CenterCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final Joystick joystick = new Joystick(0);
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();
    private final PigeonIMU pigeon;
    
    RobotContainer(PigeonIMU pigeon) {
        this.pigeon = pigeon;
        driveSubsystem.setDefaultCommand(new DriveCommand(() -> joystick.getRawAxis(1), () -> joystick.getRawAxis(0), driveSubsystem)); // default to driving from joystick input
        configureCamera();
        configureShuffleboard();
        configureButtons();
    }

    private void configureButtons() {
        new JoystickButton(joystick, 12).whileTrue(new BalanceCommand(driveSubsystem, pigeon));
        new JoystickButton(joystick, 11).onTrue(new InstantCommand(vision::limelightON));
        new JoystickButton(joystick, 10).onTrue(new InstantCommand(vision::limelightOFF));
        new JoystickButton(joystick, 3).onTrue(new CenterCommand(driveSubsystem, vision));
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
            .withProperties(Map.of("min", -27, "max", 27))
            .withWidget(BuiltInWidgets.kNumberBar);
        Shuffleboard.getTab("Vision")
            .addNumber("Target Y", vision::getTargetY)
            .withPosition(8, 0)
            .withSize(4, 3)
            .withProperties(Map.of("min", -20.5, "max", 20.5))
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
            .withSize(16, 9)
            .withWidget(BuiltInWidgets.kCameraStream);
        Shuffleboard.selectTab("Vision");
    }

    private void configureCamera() {
        UsbCamera camera = CameraServer.startAutomaticCapture(0);
        camera.setBrightness(50);
        camera.setFPS(15);
        camera.setResolution(426, 240);
    }

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            new AutoDriveCommand(24, driveSubsystem),
            new AutoDriveCommand(48, driveSubsystem)
        );
    }

}