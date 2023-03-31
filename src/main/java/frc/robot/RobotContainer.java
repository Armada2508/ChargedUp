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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Gripper;
import frc.robot.Constants.Wrist;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.ArmWristCommand;
import frc.robot.commands.arm.GripperCommand;
import frc.robot.commands.arm.WristCommand;
import frc.robot.commands.auto.AutoGripperCommand;
import frc.robot.commands.auto.ConeOnPoleCommand;
import frc.robot.commands.auto.FinishScoreCommand;
import frc.robot.commands.auto.PlacePieceCommand;
import frc.robot.commands.auto.PlacePieceCommand.Height;
import frc.robot.commands.auto.StoreCommand;
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
    private final Joystick buttonBoard = new Joystick(1);
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final GripperSubsystem gripperSubsystem = new GripperSubsystem(armSubsystem, wristSubsystem);
    private SubsystemBase[] subsystems;
    private final PigeonIMU pigeon;
    private final TimeOfFlight tof;
    private final double autoGripperCal = 0.8; // 0.8
    private double lastPitch = 0;

    public RobotContainer(PigeonIMU pigeon, TimeOfFlight tof) {
        pigeon.setYaw(0);
        this.pigeon = pigeon;
        this.tof = tof;
        this.driveSubsystem = new DriveSubsystem(pigeon);
        subsystems = new SubsystemBase[]{driveSubsystem, visionSubsystem, armSubsystem, wristSubsystem, gripperSubsystem};
        FollowTrajectory.config(0, 0, 0, Drive.ramseteB, Drive.ramseteZeta, Drive.trackWidthMeters, new PIDController(0, 0, 0), 0);
        InverseKinematics.config(Arm.jointLengthInches, Wrist.jointLengthInches);
        driveSubsystem.setDefaultCommand(new ButterySmoothDriveCommand(() -> -joystick.getRawAxis(1), () -> -joystick.getRawAxis(0),  () -> -joystick.getRawAxis(2), () -> joystick.getRawButton(12), true, driveSubsystem)); // default to driving from joystick input
        gripperSubsystem.setDefaultCommand(new AutoGripperCommand(driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem, tof));
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

    public void mapJoyButton(Command c, int b) {
        new JoystickButton(joystick, b).onTrue(c);
    }

    public void mapBoardButton(Command c, int b) {
        new JoystickButton(buttonBoard, b).onTrue(c);
    }

    public void stopEverything() {
        CommandScheduler.getInstance().cancelAll();
        driveSubsystem.stop();
        armSubsystem.stop();
        wristSubsystem.stop();
        gripperSubsystem.stop();
    }

    //! Button 12 joystick is used for slow speed.
    private void configureButtons() {
        // Close Gripper and Carry Cone
        mapJoyButton(new SequentialCommandGroup( 
            new GripperCommand(Gripper.grabCone, gripperSubsystem, armSubsystem),
            new WristCommand(Wrist.maxDegrees, 130, 130, wristSubsystem, armSubsystem)
        ), 1);

        // Open Gripper
        mapJoyButton(new GripperCommand(Gripper.open, gripperSubsystem, armSubsystem), 2); 

        // Bump Buttons
        mapJoyButton(new WristCommand(() -> wristSubsystem.getPosition() - 3, 45, 45, wristSubsystem, armSubsystem), 3);
        mapJoyButton(new ArmCommand(() -> armSubsystem.getPosition() - 3, 45, 45, armSubsystem), 4);
        mapJoyButton(new WristCommand(() -> wristSubsystem.getPosition() + 3, 45, 45, wristSubsystem, armSubsystem), 5);
        mapJoyButton(new ArmCommand(() -> armSubsystem.getPosition() + 3, 45, 45, armSubsystem), 6);

        // Place Positions
        mapJoyButton(new PlacePieceCommand(() -> Height.HIGH, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 7);
        mapJoyButton(new PlacePieceCommand(() -> Height.MID, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 9);
        mapJoyButton(new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitUntilCommand(() -> armSubsystem.getPosition() > 30),
                new GripperCommand(Gripper.open, gripperSubsystem, armSubsystem)
            ),
            new ArmWristCommand(
                new ArmCommand(94, 100, 75, armSubsystem), 
                new WristCommand(20, 130, 130, wristSubsystem, armSubsystem), 
                30, -15, armSubsystem, wristSubsystem, gripperSubsystem)
        ), 10);
        mapJoyButton(new PlacePieceCommand(() -> Height.BOTTOM, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 11);

        // Start Position - Don't press
        mapJoyButton(new SequentialCommandGroup(
            new GripperCommand(Gripper.onLimit, gripperSubsystem, armSubsystem),
            new WristCommand(Wrist.maxDegrees-10, 45, 45, wristSubsystem, armSubsystem),
            new ArmCommand(Arm.minDegrees+3, 45, 45, armSubsystem),
            new InstantCommand(() -> gripperSubsystem.setPosition(-0.3))
        ), 8);

        // Store
        mapBoardButton(new StoreCommand(armSubsystem, wristSubsystem, gripperSubsystem), 1);

        // Pickup Position
        mapBoardButton(new SequentialCommandGroup( 
            new ArmWristCommand(new ArmCommand(0, 45, 45, armSubsystem), new WristCommand(92, 45, 45, wristSubsystem, armSubsystem), -0.5, 10, armSubsystem, wristSubsystem, gripperSubsystem),
            new GripperCommand(Gripper.open, gripperSubsystem, armSubsystem)
        ), 2);

        // Calibrate All
        mapBoardButton(new SequentialCommandGroup( 
            gripperSubsystem.getCalibrateSequence(),
            wristSubsystem.getCalibrateSequence(gripperSubsystem),
            armSubsystem.getCalibrateSequence(wristSubsystem, gripperSubsystem)
        ), 3);

        // Finish Scoring
        mapBoardButton(new FinishScoreCommand(Units.inchesToMeters(18), driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 4);

        // Stop Everything
        mapBoardButton(Commands.runOnce(this::stopEverything), 5);
    }

    public void teleopInit() {
        gripperSubsystem.getCalibrateSequence().schedule();
    }

    private Command autoScoreSequence() {
        double distance = 0.6;
        double arm = ConeOnPoleCommand.armHigh;
        double wrist = ConeOnPoleCommand.wristHigh;
        double scale = 1;
        return new SequentialCommandGroup(
            gripperSubsystem.getCalibrateSequence(Gripper.onLimit + autoGripperCal, 0.3),
            wristSubsystem.getCalibrateSequence(gripperSubsystem),
            armSubsystem.getCalibrateSequence(wristSubsystem, gripperSubsystem),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new AutoDriveCommand(-distance, 2*scale, 2*scale, driveSubsystem), // go back to raise arm 
                    new WaitCommand(1),
                    new AutoDriveCommand(distance, 2*scale, 1.5*scale, driveSubsystem) // go forward to score
                ),
                new ArmWristCommand(new ArmCommand(arm, 120*scale, 70*scale, armSubsystem), new WristCommand(wrist, 130*scale, 130*scale, wristSubsystem, armSubsystem), 10, -15, armSubsystem, wristSubsystem, gripperSubsystem)
            ),
            new GripperCommand(Gripper.open, gripperSubsystem, armSubsystem),
            new WaitCommand(0.5),
            new PrintCommand("Finished Opening Gripper to score"),
            new ParallelCommandGroup( // going down
                new AutoDriveCommand(-distance, 3*scale, 2*scale, driveSubsystem),
                new GripperCommand(Gripper.onLimit, gripperSubsystem, armSubsystem),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new PrintCommand("Running Arm and Wrist down to store"),
                    new ParallelCommandGroup(
                        new ArmCommand(Arm.minDegrees, 180*scale, 120*scale, armSubsystem), 
                        new WristCommand(Wrist.maxDegrees, 150*scale, 150*scale, wristSubsystem, armSubsystem)
                    )
                )
            )
            // gripperSubsystem.getCalibrateSequence()
        );
    }

    /**
     * Calibrates, scores cone on high pole, taxis and then balances. For the middle placement. 
     */
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            // new AutoDriveCommand(2, 1.5, 0.5, driveSubsystem)
            autoScoreSequence()
            // new BalanceCommand(true, driveSubsystem, pigeon),
            // new InstantCommand(driveSubsystem::holdPosition, driveSubsystem),
            // new WaitUntilCommand(() -> { // wait until delta has stopped changing and things have calmed down
            //     double pitch = pigeon.getPitch();
            //     boolean val = Math.abs(pitch-lastPitch) <= Balance.minDelta;
            //     lastPitch = pitch;
            //     return val;
            // }),
            // new WaitCommand(1),
            // new AutoDriveCommand(0.224, 0.5, 0.2, driveSubsystem),
            // new InstantCommand(driveSubsystem::holdPosition, driveSubsystem)
        );
    }

    /**
     * Calibrates, scores cone on high pole and then taxis. For side placement.
     */
    public Command getAltAutoCommand() {
        return new SequentialCommandGroup(
            autoScoreSequence(),
            new AutoDriveCommand(-Units.inchesToMeters(120), 1.5, 0.5, driveSubsystem) 
        );
    }

}
