package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class MoveRelativeCommand extends CommandBase {

    private DoubleSupplier targetX; // in feet, pos is right, negative is left
    private DoubleSupplier targetY; // in feet, pos is forwards, negative is back
    private DoubleSupplier targetDegrees;
    private SequentialCommandGroup group;
    private final DriveSubsystem driveSubsystem;
    private final PigeonIMU pigeon;

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
    public MoveRelativeCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier degrees, DriveSubsystem driveSubsystem, PigeonIMU pigeon) {
        targetX = x;
        targetY = y;
        targetDegrees = degrees;
        this.driveSubsystem = driveSubsystem;
        this.pigeon = pigeon;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        Command firstTurn = new InstantCommand();
        double degreeOffset = 0;
        if (targetX.getAsDouble() > 0) {
            firstTurn = new AutoTurnCommand(90, driveSubsystem, pigeon);
            degreeOffset = 90;
        }
        else if (targetX.getAsDouble() < 0) {
            firstTurn = new AutoTurnCommand(-90, driveSubsystem, pigeon);
            degreeOffset = -90;
        }
        Command endTurn = new AutoTurnCommand(targetDegrees.getAsDouble() - degreeOffset, driveSubsystem, pigeon);
        group = new SequentialCommandGroup(
            new AutoDriveCommand(targetY.getAsDouble(), driveSubsystem),
            firstTurn,
            new AutoDriveCommand(targetX.getAsDouble(), driveSubsystem),
            endTurn
        );
        group.schedule();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        group.end(interrupted);
        driveSubsystem.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        return group.isFinished();
    }
}