package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Vision;
import frc.robot.commands.Driving.AutoDriveCommand;
import frc.robot.commands.Driving.AutoTurnCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class LineUpCommand extends CommandBase {

    private List<Command> commands = new ArrayList<>();
    private int currentIndex = 0;
    private Target target;
    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private PigeonIMU pigeon;

    public LineUpCommand(Target target, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, PigeonIMU pigeon) {
        this.target = target;
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.pigeon = pigeon;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        currentIndex = 0;
        commands.clear();
        if (target == Target.CONE) visionSubsystem.setPipeline(Vision.colorPipeline);
        else visionSubsystem.setPipeline(Vision.reflectionPipeline);

        double angleX = visionSubsystem.getTargetX();
        commands.add(new AutoTurnCommand(angleX, driveSubsystem, pigeon));
        commands.get(currentIndex).schedule();
    }

    @Override
    public void execute() {
        Command currentCommand = commands.get(currentIndex);
        if (currentCommand.isFinished()) {
            final double desiredDistance = (target == Target.CONE) ? Vision.coneDistance : Vision.poleDistance;
            double startingDistance = visionSubsystem.distanceFromTargetInInches(target);
            if (currentIndex == 0) commands.add(new AutoDriveCommand(startingDistance - desiredDistance, driveSubsystem));
            currentIndex++;
            commands.get(currentIndex).schedule();
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
