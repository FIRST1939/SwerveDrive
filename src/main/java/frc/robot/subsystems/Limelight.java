package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private final NetworkTable networkTable;
    private final String name;

    private final ArrayList<double[]> poses = new ArrayList<>();
    private Alliance alliance = Alliance.Invalid;

    public Limelight (String name) {

        this.networkTable = NetworkTableInstance.getDefault().getTable(name);
        this.networkTable.getEntry("ledMode").setDouble(1.0);
        this.name =  name;
    }

    @Override
    public void periodic () {

        if (this.alliance == Alliance.Invalid) { this.alliance = DriverStation.getAlliance(); }
        this.storePose(this.getPoseWithTimestamp());
    }

    public String getName () { return this.name; }
    public String getJson () { return this.networkTable.getEntry("json").getString("nothing"); }
    public double getTX () { return this.networkTable.getEntry("tx").getDouble(999999); }
    public double getTY () { return this.networkTable.getEntry("ty").getDouble(999999); }
    public double getAltTY () { return Math.min(this.networkTable.getEntry("cy0").getDouble(999999), this.networkTable.getEntry("cy1").getDouble(999999)); }

    public boolean valid () { return this.networkTable.getEntry("tv").getDouble(0.0) == 1.0; }
    public double getTA () { return this.networkTable.getEntry("ta").getDouble(0.0); }
    
    private double[] getPoseWithTimestamp () {

        double[] pose;
        double timestamp = Timer.getFPGATimestamp() - getTotalLatency() / 1000;
        double[] poseWithTimestamp = new double[7];

        if (DriverStation.getAlliance() == Alliance.Blue) {

            pose = this.networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        } else if (DriverStation.getAlliance() == Alliance.Red) {

            pose = this.networkTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        } else {

            pose = this.networkTable.getEntry("botpose").getDoubleArray(new double[6]);
        }

        for (int i = 0; i < 6; i++) { poseWithTimestamp[i] = pose[i]; }
        if (poseWithTimestamp != new double[7]) { poseWithTimestamp[6] = timestamp; }
        return poseWithTimestamp;
    }

    private void storePose (double[] pose) {

        if (pose != new double[7]) {

            this.poses.add(pose);
        }
    }

    public double[] getLatestPose3d () { return this.poses.size() == 0 ? new double[7] : this.poses.remove(0); }
    public double getTotalLatency () { return this.networkTable.getEntry("tl").getDouble(12.5) + this.networkTable.getEntry("cl").getDouble(0.0); }
    public int getPipeline () { return (int) this.networkTable.getEntry("getpipe").getDouble(-1.0); }

    public void setPipeline (int pipelineIndex) { this.networkTable.getEntry("pipeline").setDouble((double) pipelineIndex); }
    public void setCropWindow (double[] borders) { this.networkTable.getEntry("crop").setDoubleArray(borders); }
    public void setLights (int status) { this.networkTable.getEntry("ledMode").setNumber(status); }
}
