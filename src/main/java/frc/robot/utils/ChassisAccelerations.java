package frc.robot.utils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisAccelerations {

    public double ax;
    public double ay;
    public double alpha;

    public ChassisAccelerations (double ax, double ay, double alpha) {
        
        this.ax = ax;
        this.ay = ay;
        this.alpha = alpha;
    }

    public ChassisAccelerations (ChassisSpeeds newSpeed, ChassisSpeeds oldSpeed, double time) {

        this.ax = (newSpeed.vxMetersPerSecond - oldSpeed.vxMetersPerSecond) / time;
        this.ay = (newSpeed.vyMetersPerSecond - oldSpeed.vyMetersPerSecond) / time;
        this.alpha = (newSpeed.omegaRadiansPerSecond - oldSpeed.omegaRadiansPerSecond) / time;

        if (Math.abs(this.ax) > 6.0) { this.ax = 6.0 * Math.signum(this.ax); }
        if (Math.abs(this.ay) > 6.0) { this.ay = 6.0 * Math.signum(this.ay); }
        if (Math.abs(this.alpha) > 4 * Math.PI) { this.alpha = 4 * Math.PI * Math.signum(this.alpha); }
    }

    public ChassisAccelerations () {
        
        this.ax = 0.0;
        this.ay = 0.0;
        this.alpha = 0.0;
    }

}