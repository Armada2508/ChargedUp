package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Vision;
import frc.robot.Lib.util.Util;
import frc.robot.commands.Driving.AutoTurnCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class LineUpPoleCommand {

    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private PigeonIMU pigeon;

    public LineUpPoleCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PigeonIMU pigeon) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.pigeon = pigeon;
    }

    public SequentialCommandGroup getCommand() {
        return new InstantCommand(() -> visionSubsystem.setPipeline(Vision.highPolePipeline))
        .andThen(new WaitCommand(0.2))
        .andThen(new AutoTurnCommand(visionSubsystem::getTargetX, driveSubsystem, pigeon))
        .andThen(new WaitCommand(0.4))
        .andThen(() -> {
            double distance = visionSubsystem.distanceFromTargetInInches(Target.HIGH_POLE);
            visionSubsystem.setPipeline(Vision.midPolePipeline);
            double phi = visionSubsystem.getTargetX() * (Math.PI / 180);
            System.out.println("distance " + distance);
            System.out.println("phi " + phi);
            double theta = visionSubsystem.angleFromLinedUp(distance, phi);
            System.out.println("theta " + theta);
            Pair<Double, Double> currentCoords = Util.toCartesianCoordinates(distance, theta);
            System.out.println("x: " + currentCoords.getFirst() + " y: " + currentCoords.getSecond());
        }, driveSubsystem);
    }

}
