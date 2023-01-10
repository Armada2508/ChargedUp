package frc.robot;

import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

    private final Joystick joystick = new Joystick(0);
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final PigeonIMU pigeon;
    
    RobotContainer(PigeonIMU pigeon) {
        this.pigeon = pigeon;
        driveSubsystem.setDefaultCommand(new DriveCommand(() -> joystick.getRawAxis(1), () -> joystick.getRawAxis(2), driveSubsystem)); // default to driving from joystick input
        configureShuffleboard();
        configureButtons();
    }

    private void configureButtons() {
        new JoystickButton(joystick, 12).whileTrue(new BalanceCommand(driveSubsystem, pigeon));
    }

    private void configureShuffleboard() {
        Shuffleboard.getTab("Robot")
        .addDouble("Pigeon Yaw", () -> pigeon.getYaw())
        .withPosition(0, 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -720, "max", 720))
        .withSize(4, 3);
        Shuffleboard.getTab("Robot")
        .addDouble("Pigeon Pitch", () -> pigeon.getPitch())
        .withPosition(0, 3)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -90, "max", 90))
        .withSize(4, 3);
        Shuffleboard.getTab("Robot")
        .addDouble("Pigeon Roll", () -> pigeon.getRoll())
        .withPosition(0, 6)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -90, "max", 90))
        .withSize(4, 3);
    }

}
