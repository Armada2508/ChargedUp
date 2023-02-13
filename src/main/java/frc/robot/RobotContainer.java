package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Wrist;
import frc.robot.Lib.motion.FollowTrajectory;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.Auto.AutoPickupCommand;
import frc.robot.commands.Auto.PieceOnFloorCommand;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.commands.Driving.AutoTurnCommand;
import frc.robot.commands.Driving.DriveCommand;
import frc.robot.commands.Driving.SeekCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
//!import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {

    private final Joystick joystick = new Joystick(0);
    private final Joystick buttonBoard = new Joystick(1);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    //!private final PhotonSubsystem photonSubsystem = new PhotonSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
    private final DriveSubsystem driveSubsystem;
    private final PigeonIMU pigeon = new PigeonIMU(Constants.pigeonID);

    public RobotContainer() {
        pigeon.setYaw(0);
        FollowTrajectory.config(0, 0, 0, 2.0, 0.7, Drive.trackWidthMeters, new PIDController(Drive.kP, Drive.kI, Drive.kD), 0);
        InverseKinematics.config(Arm.jointLengthInches, Wrist.jointLengthInches);
        this.driveSubsystem = new DriveSubsystem(pigeon);
        driveSubsystem.setDefaultCommand(new DriveCommand(() -> joystick.getRawAxis(1)*-1 * Drive.speedMultiplier, () -> joystick.getRawAxis(0)*-1 * Drive.speedMultiplier, driveSubsystem)); // default to driving from joystick input
        if (RobotBase.isReal()) {
            configureCamera();
        }
        configureShuffleboard();
        configureButtons();
        // new CalibrateArmCommand(armSubsystem).schedule();
        // new CalibrateWristCommand(wristSubsystem).schedule();
        // new CalibrateGripperCommand(gripperSubsystem).schedule();
    }

    @SuppressWarnings("resource")
    private void configureButtons() {
        // Joystick
        final AutoPickupCommand pickup = new AutoPickupCommand(visionSubsystem, driveSubsystem, pigeon, armSubsystem, wristSubsystem, gripperSubsystem);
        new JoystickButton(joystick, 12).onTrue(runOnce(CommandScheduler.getInstance()::cancelAll)); // AutoStop 
        new JoystickButton(joystick, 11).onTrue(new BalanceCommand(driveSubsystem, pigeon));
        // new JoystickButton(buttonBoard, 12).onTrue(new PieceOnTopCommand(pickup::getPreviousTarget, Height.HIGH, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem)); 
        // new JoystickButton(buttonBoard, 11).onTrue(new PieceOnTopCommand(pickup::getPreviousTarget, Height.MID, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem)); 
        new JoystickButton(buttonBoard, 10).onTrue(new PieceOnFloorCommand(driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem));
        new JoystickButton(joystick, 9).onTrue(new SeekCommand(driveSubsystem, visionSubsystem, pigeon, Target.CONE, 12));
        new JoystickButton(joystick, 8).onTrue(pickup);
        new JoystickButton(joystick, 7).onTrue(Commands.run(() -> driveSubsystem.setPower(.25, .25), driveSubsystem));
        new JoystickButton(joystick, 6).onTrue(new AutoDriveCommand(12, driveSubsystem));
        new JoystickButton(joystick, 5).onTrue(new AutoTurnCommand(45, driveSubsystem, pigeon));
        new JoystickButton(joystick, 4).onTrue(new AutoDriveCommand(-12, driveSubsystem));
        new JoystickButton(joystick, 3).onTrue(new AutoTurnCommand(-45, driveSubsystem, pigeon));
        // Buttonboard 
        new JoystickButton(buttonBoard, 1).whileTrue(Commands.startEnd(() -> armSubsystem.setPower(0.1), () -> armSubsystem.setPower(0), armSubsystem));
        new JoystickButton(buttonBoard, 2).whileTrue(Commands.startEnd(() -> armSubsystem.setPower(-0.1), () -> armSubsystem.setPower(0), armSubsystem));
        new JoystickButton(buttonBoard, 3).whileTrue(Commands.startEnd(() -> wristSubsystem.setPower(0.1), () -> wristSubsystem.setPower(0), wristSubsystem));
        new JoystickButton(buttonBoard, 4).whileTrue(Commands.startEnd(() -> wristSubsystem.setPower(-0.1), () -> wristSubsystem.setPower(0), wristSubsystem));
        new JoystickButton(buttonBoard, 5).whileTrue(Commands.startEnd(() -> gripperSubsystem.setPower(0.1), () -> gripperSubsystem.setPower(0), gripperSubsystem));
        new JoystickButton(buttonBoard, 6).whileTrue(Commands.startEnd(() -> gripperSubsystem.setPower(-0.1), () -> gripperSubsystem.setPower(0), gripperSubsystem));
        new JoystickButton(buttonBoard, 7).whileTrue(new InstantCommand(() -> {
            armSubsystem.calibrate();
            wristSubsystem.calibrate();
            gripperSubsystem.calibrate();
        }));
        new JoystickButton(buttonBoard, 11).onTrue(new BalanceCommand(driveSubsystem, pigeon));
        new JoystickButton(buttonBoard, 12).onTrue(new InstantCommand(CommandScheduler.getInstance()::cancelAll)); // AutoStop 
        // Combo Button Example
        // new JoystickButton(joystick, 7).and(new JoystickButton(joystick, 9)).and(new JoystickButton(joystick, 11)).whileTrue(new BalanceCommand(driveSubsystem, pigeon));
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
    }

    private static int i = 0;
    public static void addPIDToShuffleBoard(PIDController pid, String name) {
        Shuffleboard.getTab("PID Tuning")
            .add(name + " PID Controller", pid)
            .withPosition(i*7, 0)
            .withWidget(BuiltInWidgets.kPIDController)
            .withSize(7, 4);
        i++;
    }

    private void configureCamera() {
        UsbCamera camera = CameraServer.startAutomaticCapture("Camera", 0);
        camera.setExposureManual(25);
        camera.setFPS(12);
        camera.setResolution(426, 240);
        Shuffleboard.getTab("Vision")
            .add(CameraServer.getVideo().getSource())
            .withPosition(0, 4)
            .withSize(4, 3)
            .withWidget(BuiltInWidgets.kCameraStream);
    }

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            new AutoTurnCommand(90, driveSubsystem, pigeon),
            new AutoDriveCommand(24, driveSubsystem),
            new AutoTurnCommand(-90, driveSubsystem, pigeon),
            new BalanceCommand(driveSubsystem, pigeon)
        );
    }

}
