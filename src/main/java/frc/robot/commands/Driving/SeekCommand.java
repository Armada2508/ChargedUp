package frc.robot.commands.driving;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class SeekCommand extends ParallelRaceGroup {
    
    private final double minimumChange = Units.inchesToMeters(6);
    private double lastDistance = 0;
    private Supplier<Target> target;
    private VisionSubsystem visionSubsystem;
    
    public SeekCommand(Supplier<Target> target, double desiredDistanceMeters, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PigeonIMU pigeon) {
        System.out.println("Creating Alt Seek Command.");
        this.target = target;
        this.visionSubsystem = visionSubsystem;
        Command drive = new AutoDriveCommand(() -> visionSubsystem.distanceFromTargetMeters(target.get()) - desiredDistanceMeters, 1, 1, driveSubsystem);
        RepeatCommand seekLoop = new SequentialCommandGroup(
            new PrintCommand("Running Alt Seek Command."),
            new AutoTurnCommand(() -> visionSubsystem.getTargetYaw(target.get()), driveSubsystem, pigeon),
            new ConditionalCommand(drive, new InstantCommand(), this::doDrive)
        ).repeatedly();
        addCommands(
            seekLoop,
            new WaitUntilCommand(() -> {
                System.out.println("Distance: " + visionSubsystem.distanceFromTargetMeters(target.get()) + " Target Distance: " + desiredDistanceMeters);
                return visionSubsystem.distanceFromTargetMeters(target.get()) <= desiredDistanceMeters;
            })
        );
    }

    private boolean doDrive() {
        boolean result = false;
        double currentDistance = visionSubsystem.distanceFromTargetMeters(target.get());
        if (Math.abs(currentDistance - lastDistance) > minimumChange) {
            result = true;
        }
        lastDistance = currentDistance;
        return result;
    }

}
