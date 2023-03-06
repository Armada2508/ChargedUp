package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class MoveRelativeCommand extends SequentialCommandGroup {

    private DoubleSupplier targetX;
    private double degreeOffset = 0;

    /**
     * Moves the robot to a position in 2D space relative to its current position
     * @param x left and right positon in 2d space to move to in meters
     * @param y forward and backward positon in 2d space to move to in meters
     * @param subsystem DriveSubsystem
     */
    public MoveRelativeCommand(double x, double y, double degrees, DriveSubsystem driveSubsystem, PigeonIMU pigeon) {
       this(() -> x, () -> y, () -> degrees, driveSubsystem, pigeon);
    }

    /**
     * Moves the robot to a position in 2D space and ending rotation relative to its current position
     * @param x left and right positon in 2d space to move to in meters
     * @param y forward and backward positon in 2d space to move to in meters
     * @param subsystem DriveSubsystem
     * @param rotation the ending rotation that the robot should be at
     */
    public MoveRelativeCommand(DoubleSupplier targetX, DoubleSupplier targetY, DoubleSupplier targetDegrees, DriveSubsystem driveSubsystem, PigeonIMU pigeon) {
        this.targetX = targetX;
        addCommands(
            new InstantCommand(this::getDegreeOffset),
            new AutoDriveCommand(targetY.getAsDouble(), driveSubsystem),
            new AutoTurnCommand(degreeOffset, driveSubsystem, pigeon),
            new AutoDriveCommand(targetX.getAsDouble(), driveSubsystem),
            new AutoTurnCommand(targetDegrees.getAsDouble() - degreeOffset, driveSubsystem, pigeon)
        );
    }

    private void getDegreeOffset() {
        if (targetX.getAsDouble() > 0) {
            degreeOffset = 90;
        }
        else if (targetX.getAsDouble() < 0) {
            degreeOffset = -90;
        }
    }

}