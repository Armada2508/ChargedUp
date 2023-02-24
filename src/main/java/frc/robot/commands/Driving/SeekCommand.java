package frc.robot.commands.Driving;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class SeekCommand extends SequentialCommandGroup {

    public SeekCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PigeonIMU pigeon, Target target, double distanceFromTargetMeters) {
        this(driveSubsystem, visionSubsystem, pigeon, () -> target, distanceFromTargetMeters);
    }

    public SeekCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PigeonIMU pigeon, Supplier<Target> target, double distanceFromTargetMeters) {
        addCommands(
            new AutoTurnCommand(() -> visionSubsystem.getTargetYaw(target.get()), driveSubsystem, pigeon),
            new DriveUntilCommand(driveSubsystem, visionSubsystem, target.get()),
            new AutoDriveCommand(() -> visionSubsystem.distanceFromTargetMeters(target.get()) - distanceFromTargetMeters, driveSubsystem)
        );
    }

    private class DriveUntilCommand extends CommandBase {

        private DriveSubsystem driveSubsystem;
        private VisionSubsystem visionSubsystem;
        private Target target;

        DriveUntilCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Target target) {
            this.driveSubsystem = driveSubsystem;
            this.visionSubsystem = visionSubsystem;
            this.target = target;
        }

        @Override
        public void initialize() {
            driveSubsystem.setPower(.5, .5);
        }

        @Override
        public void end(boolean interrupted) {
            driveSubsystem.setPower(0, 0);
        }

        @Override
        public boolean isFinished() {
            return visionSubsystem.distanceFromTargetMeters(target) <= Units.inchesToMeters(3*12);
        }

    }

}
