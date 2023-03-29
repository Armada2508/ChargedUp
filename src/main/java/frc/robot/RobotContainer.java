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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Balance;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Gripper;
import frc.robot.Constants.Wrist;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.ArmWristCommand;
import frc.robot.commands.arm.GripperCommand;
import frc.robot.commands.arm.WristCommand;
import frc.robot.commands.auto.AutoGripperCommand;
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
    // TODO: adjust auto's gripper calibrate position
    private final double autoGripperCal = 0.0;
    private double lastPitch = 0;

    public RobotContainer(PigeonIMU pigeon, TimeOfFlight tof) {
        pigeon.setYaw(0);
        this.pigeon = pigeon;
        this.tof = tof;
        this.driveSubsystem = new DriveSubsystem(pigeon);
        subsystems = new SubsystemBase[]{driveSubsystem, visionSubsystem, armSubsystem, wristSubsystem, gripperSubsystem};
        FollowTrajectory.config(0, 0, 0, Drive.ramseteB, Drive.ramseteZeta, Drive.trackWidthMeters, new PIDController(0, 0, 0), 0);
        InverseKinematics.config(Arm.jointLengthInches, Wrist.jointLengthInches);
        driveSubsystem.setDefaultCommand(new ButterySmoothDriveCommand(() -> -joystick.getRawAxis(1), () -> -joystick.getRawAxis(0),  () -> -joystick.getRawAxis(2), () -> joystick.getRawButton(4), true, driveSubsystem)); // default to driving from joystick input
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

    private void configureButtons() {
        //! Button 4 is used for slow speed.
        mapJoyButton(Commands.runOnce(this::stopEverything), 11); // Joystick Stop
        mapJoyButton(new SequentialCommandGroup( // gripper close
            new GripperCommand(Gripper.grabCone, gripperSubsystem, armSubsystem),
            new WristCommand(Wrist.maxDegrees, 45, 45, wristSubsystem, armSubsystem)
        ), 1);

        mapJoyButton(new GripperCommand(Gripper.open, gripperSubsystem, armSubsystem), 2); 

        mapJoyButton(new FinishScoreCommand(Units.inchesToMeters(18), driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 3);

        mapJoyButton(new StoreCommand(armSubsystem, wristSubsystem, gripperSubsystem), 5);

        mapJoyButton(gripperSubsystem.getCalibrateSequence(), 6);

        mapJoyButton(new SequentialCommandGroup( // pick up
            new ArmWristCommand(new ArmCommand(0, 45, 45, armSubsystem), new WristCommand(75, 45, 45, wristSubsystem, armSubsystem), -0.5, 10, armSubsystem, wristSubsystem, gripperSubsystem),
            new GripperCommand(Gripper.open, gripperSubsystem, armSubsystem)
        ), 7);

        mapJoyButton(new PlacePieceCommand(() -> Height.BOTTOM, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 8);
        mapJoyButton(new PlacePieceCommand(() -> Height.MID, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 9);
        mapJoyButton(new PlacePieceCommand(() -> Height.HIGH, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 10);

        mapJoyButton(new SequentialCommandGroup( // calibrate
            gripperSubsystem.getCalibrateSequence(),
            wristSubsystem.getCalibrateSequence(gripperSubsystem),
            armSubsystem.getCalibrateSequence(wristSubsystem, gripperSubsystem)
        ), 12);
        
        mapBoardButton(new WristCommand(() -> wristSubsystem.getPosition() + 5, 45, 45, wristSubsystem, armSubsystem), 1);
        mapBoardButton(new WristCommand(() -> wristSubsystem.getPosition() - 5, 45, 45, wristSubsystem, armSubsystem), 2);
        mapBoardButton(new ArmCommand(() -> armSubsystem.getPosition() + 3, 45, 45, armSubsystem), 3);
        mapBoardButton(new ArmCommand(() -> armSubsystem.getPosition() - 3, 45, 45, armSubsystem), 4);
        // mapButton(new SequentialCommandGroup( // Complete auto, go in front of substation, move arm, move forward. User presses button to release and back up.
        //     new AprilTagCommand(() -> Position.RIGHT, 0.5, driveSubsystem, visionSubsystem),
        //     new PlacePieceCommand(() -> Height.HIGH, armSubsystem, wristSubsystem, gripperSubsystem),
        //     new AutoDriveCommand(0.5, 1, 0.25, driveSubsystem)
        // ), 9);

        // Combo Button Example
        // new JoystickButton(joystick, 7).and(new JoystickButton(joystick, 9)).and(new JoystickButton(joystick, 11)).whileTrue(new BalanceCommand(driveSubsystem, pigeon));
    }

    public void teleopInit() {
        gripperSubsystem.getCalibrateSequence();
    }

    /**
     * Calibrates, scores cone on high pole, taxis and then balances. For the middle placement. 
     */
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
            gripperSubsystem.getCalibrateSequence(Gripper.onLimit + autoGripperCal),
            wristSubsystem.getCalibrateSequence(gripperSubsystem),
            armSubsystem.getCalibrateSequence(wristSubsystem, gripperSubsystem),
            new AutoDriveCommand(-0.3, 1, 0.25, driveSubsystem), // go back to raise arm 
            new PlacePieceCommand(() -> Height.HIGH, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 
            new AutoDriveCommand(0.3, 1, 0.25, driveSubsystem), // go forward to score
            new FinishScoreCommand(0.4, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), 
            new BalanceCommand(true, driveSubsystem, pigeon),
            new AutoDriveCommand(-0.5, 1, 0.2, driveSubsystem),
            new BalanceCommand(false, driveSubsystem, pigeon),
            new WaitUntilCommand(() -> { // wait until delta has stopped changing and things have calmed down
                double pitch = pigeon.getPitch();
                boolean val = Math.abs(pitch-lastPitch) <= Balance.minDelta;
                lastPitch = pitch;
                return val;
            }),
            new AutoDriveCommand(-0.2, 1, 0.2, driveSubsystem)
        );
    }

    /**
     * Calibrates, scores cone on high pole and then taxis. For side placement.
     */
    public Command getAltAutoCommand() {
        return new SequentialCommandGroup(
            gripperSubsystem.getCalibrateSequence(Gripper.onLimit + autoGripperCal),
            wristSubsystem.getCalibrateSequence(gripperSubsystem),
            armSubsystem.getCalibrateSequence(wristSubsystem, gripperSubsystem),
            new AutoDriveCommand(-0.3, 1, 0.25, driveSubsystem), // go back to raise arm 
            new PlacePieceCommand(() -> Height.HIGH, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem), // this doesn't go forward/backward
            new AutoDriveCommand(0.3, 1, 0.25, driveSubsystem), // go forward to score
            new FinishScoreCommand(0.4, driveSubsystem, armSubsystem, wristSubsystem, gripperSubsystem),   
            new AutoDriveCommand(-Units.inchesToMeters(152), 1, 0.25, driveSubsystem) 
        );
    }

}
