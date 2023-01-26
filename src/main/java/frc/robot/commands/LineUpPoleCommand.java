package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Lib.util.Util;
import frc.robot.commands.Driving.AutoTurnCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class LineUpPoleCommand extends CommandBase {

    private List<Command> commands = new ArrayList<>();
    private int currentIndex = 0;
    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private PigeonIMU pigeon;

    public LineUpPoleCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PigeonIMU pigeon) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.pigeon = pigeon;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        currentIndex = 0;
        commands.clear();

        double angleX = visionSubsystem.getTargetX();
        commands.add(new AutoTurnCommand(angleX, driveSubsystem, pigeon));
        commands.get(currentIndex).initialize();
    }

    @Override
    public void execute() {
        Command currentCommand = commands.get(currentIndex);
        currentCommand.execute();
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            double distance = visionSubsystem.distanceFromTargetInInches(Target.HIGH_POLE);
            double phi = visionSubsystem.getPythonData()[0];
            System.out.println(phi);
            double theta = visionSubsystem.angleFromLinedUp(distance, phi);
            Pair<Double, Double> currentCoords = Util.toCartesianCoordinates(distance, theta);
            currentIndex++;
            commands.get(currentIndex).initialize();
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        return currentIndex >= 2;
    }
   
}
