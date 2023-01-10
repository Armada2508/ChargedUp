package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem {

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public double getTargetX() {
        return table.getEntry("tx").getDouble(0);
    }
}
