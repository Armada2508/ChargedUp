package frc.robot;

import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Drive;
import frc.robot.Lib.motion.FollowTrajectory;
import frc.robot.commands.AprilTagCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.LineUpPoleCommand;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.commands.Driving.AutoTurnCommand;
import frc.robot.commands.Driving.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final Joystick joystick = new Joystick(0);
    // private final Joystick buttonBoard = new Joystick(1);
    private final VisionSubsystem vision = new VisionSubsystem();
    // private final ArmSubsystem armSubsystem = new ArmSubsystem();
    // private final WristSubsystem wristSubsystem = new WristSubsystem();
    // private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
    private final DriveSubsystem driveSubsystem;
    private final PigeonIMU pigeon = new PigeonIMU(Constants.pigeonID);

    RobotContainer() {
        FollowTrajectory.config(0, 0, 0, 2.0, 0.7, Drive.trackWidthMeters, new PIDController(Drive.kP, Drive.kI, Drive.kD), 0);
        this.driveSubsystem = new DriveSubsystem(pigeon);
        driveSubsystem.setDefaultCommand(new DriveCommand(() -> joystick.getRawAxis(1)*-1, () -> joystick.getRawAxis(2)*-1, driveSubsystem)); // default to driving from joystick input
        configureCamera();
        configureShuffleboard();
        configureButtons();
        // new CalibrateArmCommand(armSubsystem).schedule();
    }

    private void configureButtons() {
        // Joystick
        new JoystickButton(joystick, 12).onTrue(new InstantCommand(() -> { // Cancels balance command
            Command command = driveSubsystem.getCurrentCommand(); 
            if (command instanceof BalanceCommand) {
                command.cancel();
            }
        }));
        new JoystickButton(joystick, 11).onTrue(new BalanceCommand(driveSubsystem, pigeon));
        new JoystickButton(joystick, 10).onTrue(new InstantCommand(vision::limelightOFF));
        new JoystickButton(joystick, 9).onTrue(new InstantCommand(vision::limelightON));
        new JoystickButton(joystick, 8).onTrue(new AutoTurnCommand(45, driveSubsystem, pigeon));
        new JoystickButton(joystick, 7).onTrue(new AutoTurnCommand(-45, driveSubsystem, pigeon));
        new JoystickButton(joystick, 6).onTrue(new AprilTagCommand(driveSubsystem, vision));
        new JoystickButton(joystick, 4).onTrue(new LineUpPoleCommand(driveSubsystem, vision, pigeon).getCommand());
        // Buttonboard 
        /* 
        new JoystickButton(buttonBoard, 1).whileTrue(Commands.startEnd(() -> armSubsystem.setPower(0.1), () -> armSubsystem.setPower(0), armSubsystem));
        new JoystickButton(buttonBoard, 2).whileTrue(Commands.startEnd(() -> armSubsystem.setPower(-0.1), () -> armSubsystem.setPower(0), armSubsystem));
        new JoystickButton(buttonBoard, 3).whileTrue(Commands.startEnd(() -> wristSubsystem.setPower(0.1), () -> wristSubsystem.setPower(0), wristSubsystem));
        new JoystickButton(buttonBoard, 4).whileTrue(Commands.startEnd(() -> wristSubsystem.setPower(-0.1), () -> wristSubsystem.setPower(0), wristSubsystem));
        new JoystickButton(buttonBoard, 5).whileTrue(Commands.startEnd(() -> gripperSubsystem.setPower(0.1), () -> gripperSubsystem.setPower(0), gripperSubsystem));
        new JoystickButton(buttonBoard, 6).whileTrue(Commands.startEnd(() -> gripperSubsystem.setPower(-0.1), () -> gripperSubsystem.setPower(0), gripperSubsystem));
        */
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
    }

    private void configureCamera() {
        UsbCamera camera = CameraServer.startAutomaticCapture("Camera", 0);
        camera.setExposureManual(25);
        camera.setFPS(12);
        camera.setResolution(426, 240);
    }

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            new AutoTurnCommand(90, driveSubsystem, pigeon),
            new AutoDriveCommand(24, driveSubsystem),
            new AutoTurnCommand(-90, driveSubsystem, pigeon)
            // new AutoDriveCommand(48, driveSubsystem)
        );
    }

}
