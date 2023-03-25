package frc.robot;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Wrist;
import frc.robot.commands.driving.AutoDriveCommand;
import frc.robot.commands.driving.ButterySmoothDriveCommand;
import frc.robot.lib.motion.FollowTrajectory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

@SuppressWarnings("unused")
public class RobotContainer {

    private final Joystick joystick = new Joystick(0);
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final GripperSubsystem gripperSubsystem = new GripperSubsystem(armSubsystem, wristSubsystem);
    private SubsystemBase[] subsystems;
    private final PigeonIMU pigeon;
    private final TimeOfFlight tof;

    public RobotContainer(PigeonIMU pigeon, TimeOfFlight tof) {
        pigeon.setYaw(0);
        this.pigeon = pigeon;
        this.tof = tof;
        this.driveSubsystem = new DriveSubsystem(pigeon);
        subsystems = new SubsystemBase[]{driveSubsystem, visionSubsystem, armSubsystem, wristSubsystem, gripperSubsystem};
        FollowTrajectory.config(0, 0, 0, 0, 0, Drive.trackWidthMeters, new PIDController(0, 0, 0), 0);
        InverseKinematics.config(Arm.jointLengthInches, Wrist.jointLengthInches);
        driveSubsystem.setDefaultCommand(new ButterySmoothDriveCommand(() -> -joystick.getRawAxis(1), () -> -joystick.getRawAxis(0),  () -> -joystick.getRawAxis(2), () -> joystick.getRawButton(4), true, driveSubsystem)); // default to driving from joystick input
        configureShuffleboard();
        configureButtons();
        // logSubsystems();
    }

    @SuppressWarnings("unchecked")
    private void logSubsystems() {
        try {
            final Field fieldIndex = SequentialCommandGroup.class.getDeclaredField("m_currentCommandIndex");
            fieldIndex.setAccessible(true);
            final Field fieldCommands = SequentialCommandGroup.class.getDeclaredField("m_commands");
            fieldCommands.setAccessible(true);
            SubsystemBase loggerSubsystem = new SubsystemBase() {};
            loggerSubsystem.setDefaultCommand(Commands.run(() -> {
                System.out.println("\nDEBUG: Subsystem Logger");
                for (int i = 0; i < subsystems.length; i++) {
                    String name = "None";
                    Command command = subsystems[i].getCurrentCommand();
                    if (command != null) {
                        name = command.getName();
                        if (command instanceof SequentialCommandGroup) {
                            try {
                                List<Command> list = (List<Command>) fieldCommands.get(command);
                                name += " - " + list.get(fieldIndex.getInt(command)).getName();
                            } catch (IllegalArgumentException | IllegalAccessException e) {
                                e.printStackTrace();
                            }
                        }
                    }
                    System.out.println(subsystems[i].getName() + ": " + name);
                }
            }, loggerSubsystem));
        } catch (NoSuchFieldException | SecurityException e) {
            e.printStackTrace();
        }
    }

    public void mapButton(Command c, int b) {
        new JoystickButton(joystick, b).onTrue(c);
    }

    public void stopEverything() {
        CommandScheduler.getInstance().cancelAll();
        driveSubsystem.stop();
        armSubsystem.stop();
        wristSubsystem.stop();
        gripperSubsystem.stop();
    }

    private void configureButtons() {
        //! Button 4 is used for slow speed.
        mapButton(Commands.runOnce(this::stopEverything), 11); // Joystick Stop
        /*
        mapButton(new GripperCommand(Gripper.grabCone, gripperSubsystem), 1); // gripper close
        mapButton(new GripperCommand(0, gripperSubsystem), 2); // gripper open

        mapButton(new SequentialCommandGroup( // pick up
            new ArmCommand(60, 45, 45, armSubsystem),
            new WristCommand(-45, 180, 180, wristSubsystem)
        ), 7);

        mapButton(new SequentialCommandGroup( // place down
            new ArmCommand(102, 45, 45, armSubsystem),
            new WristCommand(0, 180, 180, wristSubsystem)
        ), 8);

        mapButton(new SequentialCommandGroup( // store
            new WristCommand(Wrist.maxDegrees, 180, 180, wristSubsystem),
            new ArmCommand(5, 45, 45, armSubsystem)
        ), 9);

        mapButton(new SequentialCommandGroup( // calibrate
            gripperSubsystem.getCalibrateSequence(),
            wristSubsystem.getCalibrateSequence(gripperSubsystem),
            armSubsystem.getCalibrateSequence(wristSubsystem, gripperSubsystem)
        ), 12);
        */
        mapButton(gripperSubsystem.getCalibrateSequence(), 12);
        // mapButton(new RunCommand(() -> driveSubsystem.setVelocity(0.25, 0.25), driveSubsystem), 6);
        // mapButton(new RunCommand(() -> driveSubsystem.setVelocity(0.5, 0.5), driveSubsystem), 7);
        // mapButton(new RunCommand(() -> driveSubsystem.setVelocity(1, 1), driveSubsystem), 8);
        // mapButton(new RunCommand(() -> driveSubsystem.setVelocity(2, 2), driveSubsystem), 9);
        // mapButton(new RunCommand(() -> driveSubsystem.setVelocity(3, 3), driveSubsystem), 10);
        // mapButton(new AprilTagCommand(() -> Position.CENTER, driveSubsystem, visionSubsystem), 12);
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

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            // new BalanceCommand(false, driveSubsystem, pigeon),
            new AutoDriveCommand(1, 1, 0.2, driveSubsystem)
            // new AutoDriveCommand(1, 1, 0.2, driveSubsystem)
            // new AutoDriveCommand(-0.2, 1, 0.2, driveSubsystem)
            

        );
    }

}
