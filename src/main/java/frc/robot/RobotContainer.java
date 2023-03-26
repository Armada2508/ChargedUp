package frc.robot;

import java.lang.reflect.Field;
import java.util.List;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Wrist;
import frc.robot.commands.auto.AprilTagCommand;
import frc.robot.commands.auto.AprilTagCommand.Position;
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
        FollowTrajectory.config(0, 0, 0, Drive.ramseteB, Drive.ramseteZeta, Drive.trackWidthMeters, new PIDController(0, 0, 0), 0);
        InverseKinematics.config(Arm.jointLengthInches, Wrist.jointLengthInches);
        driveSubsystem.setDefaultCommand(new ButterySmoothDriveCommand(() -> -joystick.getRawAxis(1), () -> -joystick.getRawAxis(0),  () -> -joystick.getRawAxis(2), () -> joystick.getRawButton(4), true, driveSubsystem)); // default to driving from joystick input
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

    public void mapWhileButton(Command c, int b) {
        new JoystickButton(joystick, b).whileTrue(c);
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
        mapButton(new SequentialCommandGroup( // gripper close
            new GripperCommand(Gripper.grabCone, gripperSubsystem, armSubsystem),
            new WristCommand(Wrist.maxDegrees, 45, 45, wristSubsystem)
        ), 1);

        mapButton(new GripperCommand(Gripper.open, gripperSubsystem, armSubsystem), 2); 

        mapButton(new FinishScoreCommand(Units.inchesToMeters(-18), driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 3);

        mapButton(new StoreCommand(armSubsystem, wristSubsystem, gripperSubsystem), 5);

        mapButton(new SequentialCommandGroup( // pick up
            new ArmWristCommand(new ArmCommand(0, 45, 45, armSubsystem), new WristCommand(75, 45, 45, wristSubsystem), -0.5, 10, armSubsystem, wristSubsystem, gripperSubsystem),
            new GripperCommand(Gripper.open, gripperSubsystem, armSubsystem)
        ), 7);

        mapButton(new PlacePieceCommand(() -> Height.BOTTOM, armSubsystem, wristSubsystem, gripperSubsystem), 8);
        mapButton(new PlacePieceCommand(() -> Height.MID, armSubsystem, wristSubsystem, gripperSubsystem), 9);
        mapButton(new PlacePieceCommand(() -> Height.HIGH, armSubsystem, wristSubsystem, gripperSubsystem), 10);

        mapButton(new SequentialCommandGroup( // calibrate
            gripperSubsystem.getCalibrateSequence(),
            wristSubsystem.getCalibrateSequence(gripperSubsystem),
            armSubsystem.getCalibrateSequence(wristSubsystem, gripperSubsystem)
        ), 12);
        */
        mapButton(new RunCommand(() -> driveSubsystem.setVelocity(0.25, 0.25), driveSubsystem), 6);
        mapButton(new RunCommand(() -> driveSubsystem.setVelocity(0.5, 0.5), driveSubsystem), 7);
        mapButton(new RunCommand(() -> driveSubsystem.setVelocity(1, 1), driveSubsystem), 8);
        mapButton(new RunCommand(() -> driveSubsystem.setVelocity(2, 2), driveSubsystem), 9);
        mapButton(new RunCommand(() -> driveSubsystem.setVelocity(3, 3), driveSubsystem), 10);
        mapButton(new AprilTagCommand(() -> Position.CENTER, driveSubsystem, visionSubsystem), 12);
        // Combo Button Example
        // new JoystickButton(joystick, 7).and(new JoystickButton(joystick, 9)).and(new JoystickButton(joystick, 11)).whileTrue(new BalanceCommand(driveSubsystem, pigeon));
    }

    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            // new BalanceCommand(false, driveSubsystem, pigeon),
            new AutoDriveCommand(Units.inchesToMeters(376.98), 0.25, 0.25, driveSubsystem)
            // new AutoDriveCommand(1, 1, 0.2, driveSubsystem)
            // new AutoDriveCommand(-0.2, 1, 0.2, driveSubsystem)
            

        );
    }

}
