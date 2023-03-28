package frc.robot.commands.driving;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Target;

public class ConeTurnCommand extends CommandBase {
    
    private PIDController controller = new PIDController(0.0004, 0, 0);
    private final int pixelTolerance = 30;
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public ConeTurnCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        controller.setSetpoint(0);
        controller.setTolerance(pixelTolerance);
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        controller.reset();
        if (!visionSubsystem.hasTarget(Target.CONE)) {
            cancel();
            return;
        }
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget(Target.CONE)) {
            double speed = controller.calculate(visionSubsystem.getTargetX(Target.CONE));
            System.out.println(speed);
            speed = MathUtil.clamp(speed, -Drive.maxTurnSpeed, Drive.maxTurnSpeed);
            driveSubsystem.setPower(-speed, speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

}
