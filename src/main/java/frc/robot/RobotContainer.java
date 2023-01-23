package frc.robot;

import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.cameraserver.CameraServer;
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
    // Test Comment
    private final Joystick joystick = new Joystick(0);
    private final VisionSubsystem vision = new VisionSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final DriveSubsystem driveSubsystem;
    private final PigeonIMU pigeon = new PigeonIMU(Constants.pigeonID);
    // ! Auto Turn Command Drift
    RobotContainer() {
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
        new JoystickButton(joystick, 5).whileTrue(Commands.startEnd(() -> armSubsystem.setPower(0.1), () -> armSubsystem.setPower(0), armSubsystem));
        new JoystickButton(joystick, 3).whileTrue(Commands.startEnd(() -> armSubsystem.setPower(-0.1), () -> armSubsystem.setPower(0), armSubsystem));
        new JoystickButton(joystick, 6).onTrue(new LineUpCommand(Target.CONE, driveSubsystem, vision, pigeon));
        new JoystickButton(joystick, 4).onTrue(new LineUpCommand(Target.HIGH_POLE, driveSubsystem, vision, pigeon));
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
        Shuffleboard.selectTab("Vision");
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
