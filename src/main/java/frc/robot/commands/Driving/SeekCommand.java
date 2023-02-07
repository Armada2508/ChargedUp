package frc.robot.commands.Driving;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.PhotonSubsystem.Target;

public class SeekCommand extends SequentialCommandGroup {

    public SeekCommand(DriveSubsystem driveSubsystem, PhotonSubsystem photonSubsystem, PigeonIMU pigeon, Target target, double distanceFromTargetInches) {
        addCommands(
            new AutoTurnCommand(photonSubsystem::getTargetYaw, driveSubsystem, pigeon),
            new DriveUntilCommand(driveSubsystem, photonSubsystem, target),
            new AutoDriveCommand(() -> photonSubsystem.getDistanceToTargetInches(target) - distanceFromTargetInches, driveSubsystem)
        );
    }

    private class DriveUntilCommand extends CommandBase {

        private DriveSubsystem driveSubsystem;
        private PhotonSubsystem photonSubsystem;
        private Target target;

        DriveUntilCommand(DriveSubsystem driveSubsystem, PhotonSubsystem photonSubsystem, Target target) {
            this.driveSubsystem = driveSubsystem;
            this.photonSubsystem = photonSubsystem;
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
            return photonSubsystem.getDistanceToTargetInches(target) <= 3*12;
        }

    }

}
