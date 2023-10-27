package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;

public class PoseEstimator extends SubsystemBase {

    private final SwerveDrivePoseEstimator swervPoseEstimator;
    private final SwerveDrive swerveDrive;
    private final Limelight limelight;

    private boolean initializedPose = true;

    public PoseEstimator(SwerveDrive swerveDrive, Limelight limelight, Pose2d initialPose) {

        this.swerveDrive = swerveDrive;
        this.limelight = limelight;

        this.swervPoseEstimator = new SwerveDrivePoseEstimator(
            Constants.SwerveModuleConstants.SWERVE_DRIVE_KINEMATICS,
            this.swerveDrive.getGyro(),
            this.swerveDrive.getModulePositions(),
            initialPose,
            VecBuilder.fill(0.229, 0.229, 0.0), 
            VecBuilder.fill(5, 5, 5)
        );
    }

    @Override
    public void periodic () { this.updatePoseEstimator(); }

    private void updatePoseEstimator () {

        double[] visionMeasurement = this.limelight.getLatestPose3d();
        double timestamp = visionMeasurement[6];

        double velocity = MathUtils.pythagorean(this.swerveDrive.getChassisSpeed().vxMetersPerSecond, this.swerveDrive.getChassisSpeed().vyMetersPerSecond);
        double angularVelocity = this.swerveDrive.getChassisSpeed().omegaRadiansPerSecond;
        
        Pose2d currentPose = getPose();
        Pose2d visionPose = new Pose2d(
            new Translation2d(visionMeasurement[0], visionMeasurement[1]),
            new Rotation2d(visionMeasurement[5])
        );

        this.swervPoseEstimator.updateWithTime(Timer.getFPGATimestamp(), this.swerveDrive.getGyro(), this.swerveDrive.getModulePositions());

        if (((currentPose.getTranslation().getDistance(visionPose.getTranslation()) <= Constants.LimelightConstants.kPoseErrorAcceptance || !this.initializedPose) && 
            visionMeasurement != new double[7] && visionPose.getTranslation().getDistance(currentPose.getTranslation()) >= 0.05 && 
            velocity <= 3.0 && angularVelocity <= 0.5 * Math.PI && visionPose.getTranslation().getX() <= 5.0)) {
                
            if (this.initializedPose) {

                if (this.limelight.valid()){

                    this.swervPoseEstimator.addVisionMeasurement(visionPose, timestamp, VecBuilder.fill(5.0, 5.0, 5.0));
                }
            }
        }
    }

    public Pose2d getPose () { return this.swervPoseEstimator.getEstimatedPosition(); }

    public Pose2d getPose (boolean allianceOriented) { return this.swervPoseEstimator.getEstimatedPosition(); }

    public boolean inside (Translation2d[] bounds, boolean onEdge) {

        Pose2d currentPose = this.getPose();
        double xMin = Math.min(bounds[0].getX(), bounds[1].getX());
        double xMax = Math.max(bounds[0].getX(), bounds[1].getX());
        double yMin = Math.min(bounds[0].getY(), bounds[1].getY());
        double yMax = Math.max(bounds[0].getY(), bounds[1].getY());

        return (
            (currentPose.getX() > xMin && currentPose.getX() < xMax) || (onEdge && (currentPose.getX() >= xMin && currentPose.getX() <= xMax)) && 
            (currentPose.getY() > yMin && currentPose.getY() < yMax) || (onEdge && (currentPose.getY() >= yMin && currentPose.getY() <= yMax))
        );
    }

    public void resetOdometry (Pose2d pose) {

        this.swerveDrive.resetOdometry(new Pose2d(pose.getTranslation(), pose.getRotation()));
        this.swervPoseEstimator.resetPosition(this.swerveDrive.getGyro().times(-1.0), this.swerveDrive.getModulePositions(), pose);
    }
}
