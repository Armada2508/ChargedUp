package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.commands.Driving.AutoTurnCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class AltSeekCommand extends ParallelRaceGroup {
    
    private final double yawDeadbandDeg = 0.5;
    private final double distanceDeadbandMeters = Units.inchesToMeters(0.5);
    private Supplier<Target> target;
    private VisionSubsystem visionSubsystem;
    
    public AltSeekCommand(Supplier<Target> target, double distanceFromTargetMeters, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PigeonIMU pigeon) {
        this.target = target;
        this.visionSubsystem = visionSubsystem;
        SequentialCommandGroup group = new SequentialCommandGroup(
            new ConditionalCommand(new AutoTurnCommand(() -> visionSubsystem.getTargetYaw(target.get()), driveSubsystem, pigeon), new InstantCommand(), this::doTurn),
            new AutoDriveCommand(() -> visionSubsystem.distanceFromTargetMeters(target.get()) - distanceFromTargetMeters, driveSubsystem)
        );
        addCommands(
            new RepeatCommand(group),
            new WaitUntilCommand(() -> visionSubsystem.distanceFromTargetMeters(target.get()) <= distanceFromTargetMeters + distanceDeadbandMeters)
        );
    }

    private boolean doTurn() {
        return Math.abs(visionSubsystem.getTargetYaw(target.get())) > yawDeadbandDeg;
    }

}
