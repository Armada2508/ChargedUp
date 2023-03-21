package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class MoveRelativeCommand extends SequentialCommandGroup {

    private final double velocity = 0.5;
    private final double acceleration = 0.25;

    /**
     * Moves the robot to a position in 2D space and ending rotation relative to its current position
     * @param targetX left and right positon in 2d space to move to in meters
     * @param targetY forward and backward poiton in 2d space to move to in meters
     * @param finalAngleRad angle that you want the robot to end its position at relative to its starting angle in radians
     * @param driveSubsystem driveSubsystem
     * @param pigeon pigeon
     */
    public MoveRelativeCommand(double targetX, double targetY, double finalAngleRad, DriveSubsystem driveSubsystem, PigeonIMU pigeon) {
       this(() -> targetX, () -> targetY, () -> finalAngleRad, driveSubsystem, pigeon);
    }

    /**
     * Moves the robot to a position in 2D space and ending rotation relative to its current position
     * @param targetX left and right positon in 2d space to move to in meters
     * @param targetY forward and backward poiton in 2d space to move to in meters
     * @param finalAngleRad angle that you want the robot to end its position at relative to its starting angle in radians
     * @param driveSubsystem driveSubsystem
     * @param pigeon pigeon
     */
    public MoveRelativeCommand(DoubleSupplier targetX, DoubleSupplier targetY, DoubleSupplier finalAngleRad, DriveSubsystem driveSubsystem, PigeonIMU pigeon) {
        addCommands(
            new PrintCommand("First Turn: "  + Math.toDegrees(Math.atan2(targetX.getAsDouble(), targetY.getAsDouble())) + " Distance: "  + 
            Math.hypot(targetX.getAsDouble(), targetY.getAsDouble()) + " Skew: " + Math.toDegrees(finalAngleRad.getAsDouble()) + " Second Turn: " + 
            Math.toDegrees(-Math.atan2(targetX.getAsDouble(), targetY.getAsDouble()) + finalAngleRad.getAsDouble())),
            new AutoTurnCommand(() -> Math.toDegrees(Math.atan2(targetX.getAsDouble(), targetY.getAsDouble())), driveSubsystem, pigeon),
            new AutoDriveCommand(() -> Math.hypot(targetX.getAsDouble(), targetY.getAsDouble()), velocity, acceleration, driveSubsystem),
            new AutoTurnCommand(() -> Math.toDegrees(-Math.atan2(targetX.getAsDouble(), targetY.getAsDouble()) + finalAngleRad.getAsDouble()), driveSubsystem, pigeon)
        );
    }

}