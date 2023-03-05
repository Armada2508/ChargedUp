package frc.robot;

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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Wrist;
import frc.robot.Lib.motion.FollowTrajectory;
import frc.robot.commands.Arm.ArmCommand;
import frc.robot.commands.Driving.ButterySmoothDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {

    private final Joystick joystick = new Joystick(0);
    private final Joystick buttonBoard = new Joystick(1);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final GripperSubsystem gripperSubsystem = new GripperSubsystem(armSubsystem, wristSubsystem);
    private final DriveSubsystem driveSubsystem;
    private final PigeonIMU pigeon = new PigeonIMU(Constants.pigeonID);

    public RobotContainer() {
        pigeon.setYaw(0);
        FollowTrajectory.config(0.31, 1.95, 0.35, 2.0, 0.7, Drive.trackWidthMeters, new PIDController(0.25, 0, 0), 0.875);
        InverseKinematics.config(Arm.jointLengthInches, Wrist.jointLengthInches);
        this.driveSubsystem = new DriveSubsystem(pigeon);
        driveSubsystem.setDefaultCommand(new ButterySmoothDriveCommand(() -> joystick.getRawAxis(1)*-1 * Drive.speedMultiplier, () -> joystick.getRawAxis(0)*-1,  () -> joystick.getRawAxis(2),driveSubsystem)); // default to driving from joystick input
        if (RobotBase.isReal()) {
            configureCamera();
        }
        configureShuffleboard();
        configureButtons();
    }

    // private Command testTrajectory() {
    //     return new MoveRelativeCommand(Units.inchesToMeters(12), Units.inchesToMeters(12), 0, driveSubsystem, pigeon);
    // }

    public void mapButton(Command c, int button) {
        new JoystickButton(joystick, button).onTrue(c);
    }

    private void panicButton() {
        CommandScheduler.getInstance().cancelAll();
        driveSubsystem.setPower(0, 0);
        armSubsystem.setPower(0);
        wristSubsystem.setPower(0);
        gripperSubsystem.setPower(0);
    }

    private void configureButtons() {
        // Joystick
        mapButton(new ArmCommand(45, armSubsystem), 10);
        // ? final AutoPickupCommand pickup = new AutoPickupCommand(visionSubsystem, driveSubsystem, pigeon, armSubsystem, wristSubsystem, gripperSubsystem);
        new JoystickButton(joystick, 11).onTrue(Commands.runOnce(this::panicButton)); // AutoStop 
        // new JoystickButton(joystick, 11).onTrue(new BalanceCommand(driveSubsystem, pigeon));
        // new JoystickButton(buttonBoard, 12).onTrue(new PieceOnTopCommand(pickup::getPreviousTarget, Height.HIGH, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem)); 
        // new JoystickButton(buttonBoard, 11).onTrue(new PieceOnTopCommand(pickup::getPreviousTarget, Height.MID, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem)); 
        // new JoystickButton(buttonBoard, 10).onTrue(new PieceOnFloorCommand(driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem));
        // ? new JoystickButton(joystick, 9).and(new JoystickButton(joystick, 10)).onTrue(new PositionAndPlaceCommand(joystick, pickup::getPreviousTarget, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, visionSubsystem, pigeon));
        // new JoystickButton(joystick, 9).onTrue(new SeekCommand(Target.CONE, Units.inchesToMeters(12), driveSubsystem, visionSubsystem, pigeon));
        // new JoystickButton(joystick, 8).onTrue(pickup);
        // new JoystickButton(joystick, 7).onTrue(new AltSeekCommand(() -> Target.CONE, Units.inchesToMeters(12), driveSubsystem, visionSubsystem, pigeon));
        // new JoystickButton(joystick, 7).onTrue(Commands.run(() -> driveSubsystem.setPower(.25, .25), driveSubsystem));
        // new JoystickButton(joystick, 7).onTrue(new AprilTagCommand(new Pose2d(Units.inchesToMeters(24), 0, new Rotation2d()), driveSubsystem, visionSubsystem, pigeon));
        // new JoystickButton(joystick, 6).onTrue(new AutoDriveCommand(Units.inchesToMeters(36), driveSubsystem));
        // new JoystickButton(joystick, 5).onTrue(new AutoTurnCommand(45, driveSubsystem, pigeon));
        // new JoystickButton(joystick, 4).onTrue(new AutoDriveCommand(Units.inchesToMeters(-36), driveSubsystem));
        // new JoystickButton(joystick, 3).onTrue(new AutoTurnCommand(-45, driveSubsystem, pigeon));
        // new JoystickButton(joystick, 3).onTrue(Commands.startEnd(() -> driveSubsystem.setPower(0.15, 0.15), () -> driveSubsystem.setPower(0, 0), driveSubsystem));
        // new JoystickButton(joystick, 5).onTrue(runOnce(() -> new AutoTurnCommand(visionSubsystem::getTargetYaw, driveSubsystem, pigeon)));
        // Buttonboard 
        // new JoystickButton(buttonBoard, 1).whileTrue(Commands.startEnd(() -> armSubsystem.setPower(0.1), () -> armSubsystem.setPower(0), armSubsystem));
        // new JoystickButton(buttonBoard, 2).whileTrue(Commands.startEnd(() -> armSubsystem.setPower(-0.1), () -> armSubsystem.setPower(0), armSubsystem));
        // new JoystickButton(buttonBoard, 3).whileTrue(Commands.startEnd(() -> wristSubsystem.setPower(0.1), () -> wristSubsystem.setPower(0), wristSubsystem));
        // new JoystickButton(buttonBoard, 4).whileTrue(Commands.startEnd(() -> wristSubsystem.setPower(-0.1), () -> wristSubsystem.setPower(0), wristSubsystem));
        // new JoystickButton(buttonBoard, 5).whileTrue(Commands.startEnd(() -> gripperSubsystem.setPower(0.1), () -> gripperSubsystem.setPower(0), gripperSubsystem));
        // new JoystickButton(buttonBoard, 6).whileTrue(Commands.startEnd(() -> gripperSubsystem.setPower(-0.1), () -> gripperSubsystem.setPower(0), gripperSubsystem));
        // new JoystickButton(buttonBoard, 7).whileTrue(new InstantCommand(() -> {
        //     armSubsystem.calibrate(Arm.minDegrees);
        //     wristSubsystem.calibrate(Wrist.minDegrees);
        //     gripperSubsystem.calibrate(0);
        // }));
        // new JoystickButton(buttonBoard, 8).onTrue(new SequentialCommandGroup(
        //     new CalibrateArmCommand(armSubsystem),
        //     new CalibrateWristCommand(wristSubsystem),
        //     new CalibrateGripperCommand(gripperSubsystem)
        // )); 
        // new JoystickButton(buttonBoard, 11).onTrue(new BalanceCommand(driveSubsystem, pigeon));
        // new JoystickButton(buttonBoard, 12).onTrue(new InstantCommand(CommandScheduler.getInstance()::cancelAll)); // AutoStop 
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

    // private static int i = 0;
    public static void addPIDToShuffleBoard(PIDController pid, String name) {
        int i = 1;
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
        return new InstantCommand();
        // return new SequentialCommandGroup(
            // new AutoTurnCommand(90, driveSubsystem, pigeon),
            // new AutoDriveCommand(24, driveSubsystem),
            // new AutoTurnCommand(-90, driveSubsystem, pigeon),
            // new BalanceCommand(driveSubsystem, pigeon)
        // );
    }

}
