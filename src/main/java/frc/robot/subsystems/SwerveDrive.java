package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.opencv.core.Mat;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.ChassisAccelerations;
import frc.robot.utils.MathUtils;

public class SwerveDrive extends SubsystemBase {

    private boolean fieldOriented = false;

    private double maintainAngle = 0.0;
    private double timeSinceRotation = 0.0;
    private double lastRotationTime = 0.0;
    private double timeSinceDrive = 0.0;
    private double lastDriveTime = 0.0;

    private final PIDController maintainAnglePID = new PIDController(
            Constants.SwerveModuleConstants.MAINTAIN_ANGLE_PID[0],
            Constants.SwerveModuleConstants.MAINTAIN_ANGLE_PID[1],
            Constants.SwerveModuleConstants.MAINTAIN_ANGLE_PID[2]);

    private final Timer maintainAngleTimer = new Timer();

    private SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(8.0);
    private SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(8.0);
    private SlewRateLimiter slewRateLimiterZ = new SlewRateLimiter(16.0);

    private final SwerveModule frontLeftModule = new SwerveModule(
            Constants.SwerveModuleConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Constants.SwerveModuleConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
            Constants.SwerveModuleConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
            Constants.SwerveModuleConstants.FRONT_LEFT_MODULE_ENCODER_OFFSET);

    private final SwerveModule frontRightModule = new SwerveModule(
            Constants.SwerveModuleConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.SwerveModuleConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
            Constants.SwerveModuleConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
            Constants.SwerveModuleConstants.FRONT_RIGHT_MODULE_ENCODER_OFFSET);

    private final SwerveModule backLeftModule = new SwerveModule(
            Constants.SwerveModuleConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Constants.SwerveModuleConstants.BACK_LEFT_MODULE_STEER_MOTOR,
            Constants.SwerveModuleConstants.BACK_LEFT_MODULE_STEER_ENCODER,
            Constants.SwerveModuleConstants.BACK_LEFT_MODULE_ENCODER_OFFSET);

    private final SwerveModule backRightModule = new SwerveModule(
            Constants.SwerveModuleConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Constants.SwerveModuleConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
            Constants.SwerveModuleConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
            Constants.SwerveModuleConstants.BACK_RIGHT_MODULE_ENCODER_OFFSET);

    private AHRS navX;
    private final SwerveDriveOdometry swerveDriveOdometry;
    private final SwerveDriveOdometry autoOdometry;

    private ChassisSpeeds lastDriveSpeed = new ChassisSpeeds();
    private ChassisAccelerations driveAccelerations = new ChassisAccelerations();

    private final double[] latestSlewRates = { 0.0, 0.0, 0.0 };
    private SwerveModuleState[] desiredModuleStates = Constants.SwerveModuleConstants.SWERVE_DRIVE_KINEMATICS
            .toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));

    public SwerveDrive (AHRS navX) {

        this.navX = navX;

        this.swerveDriveOdometry = new SwerveDriveOdometry(
                Constants.SwerveModuleConstants.SWERVE_DRIVE_KINEMATICS,
                Rotation2d.fromDegrees(this.navX.getYaw()),
                this.getModulePositions());

        this.autoOdometry = new SwerveDriveOdometry(
                Constants.SwerveModuleConstants.SWERVE_DRIVE_KINEMATICS,
                Rotation2d.fromDegrees(this.navX.getYaw()),
                this.getModulePositions());

        this.maintainAngleTimer.reset();
        this.maintainAngleTimer.start();
        this.maintainAnglePID.enableContinuousInput(-Math.PI, Math.PI);

        this.swerveDriveOdometry.resetPosition(Rotation2d.fromDegrees(this.navX.getYaw()), this.getModulePositions(), new Pose2d());
        this.navX.reset();
        this.navX.calibrate();
    }

    public void drive (double xSpeed, double ySpeed, double rotation, boolean fieldRelative, boolean keepAngle) {

        if ((!fieldRelative && this.fieldOriented)) {

            this.fieldOriented = false;
            this.slewRateLimiterX = new SlewRateLimiter(8.0, xSpeed);
            this.slewRateLimiterY = new SlewRateLimiter(8.0, ySpeed);
        } else if (fieldRelative && !this.fieldOriented) {

            this.fieldOriented = true;
            this.slewRateLimiterX = new SlewRateLimiter(8.0, xSpeed);
            this.slewRateLimiterY = new SlewRateLimiter(8.0, ySpeed);
        }

        xSpeed = this.slewRateLimiterX.calculate(xSpeed);
        ySpeed = this.slewRateLimiterY.calculate(ySpeed);
        rotation = this.slewRateLimiterZ.calculate(rotation);

        this.latestSlewRates[0] = xSpeed;
        this.latestSlewRates[1] = ySpeed;
        this.latestSlewRates[2] = rotation;

        if (keepAngle) {

            rotation = this.performMaintainAngle(xSpeed, ySpeed, rotation);
        }

        if (fieldRelative) {

            this.setModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, Rotation2d.fromDegrees(this.navX.getYaw())));
        } else {

            this.setModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rotation));
        }
    }

    @Override
    public void periodic () {

        this.driveAccelerations = new ChassisAccelerations(this.getChassisSpeed(), this.lastDriveSpeed, 0.020);
        this.lastDriveSpeed = this.getChassisSpeed();

        double xSpeed = this.getChassisSpeed().vxMetersPerSecond;
        double ySpeed = this.getChassisSpeed().vyMetersPerSecond;

        double speed = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
        SmartDashboard.putNumber("Speed [m/s]", speed);

        SmartDashboard.putNumber("Front Left Encoder", this.frontLeftModule.getTurnEncoder());
        SmartDashboard.putNumber("Front Right Encoder", this.frontRightModule.getTurnEncoder());
        SmartDashboard.putNumber("Back Left Encoder", this.backLeftModule.getTurnEncoder());
        SmartDashboard.putNumber("Back Right Encoder", this.backRightModule.getTurnEncoder());

        this.updateOdometry();
        this.getPose();
    }

    public void setModuleStates (SwerveModuleState[] desiredModuleStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, Constants.SwerveModuleConstants.MAX_VELOCITY);

        this.frontLeftModule.setDesiredState(desiredModuleStates[0]);
        this.frontRightModule.setDesiredState(desiredModuleStates[1]);
        this.backLeftModule.setDesiredState(desiredModuleStates[2]);
        this.backRightModule.setDesiredState(desiredModuleStates[3]);
    }

    public void setModuleStates (ChassisSpeeds chassisSpeeds) {

        SwerveModuleState[] desiredModuleStates = Constants.SwerveModuleConstants.SWERVE_DRIVE_KINEMATICS
                .toSwerveModuleStates(this.secondOrderKinematics(chassisSpeeds));
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, Constants.SwerveModuleConstants.MAX_VELOCITY);
        this.desiredModuleStates = desiredModuleStates;

        if (Math.abs(chassisSpeeds.vxMetersPerSecond) < 0.01 &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) < 0.01 &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 0.0075) {

            desiredModuleStates[0] = Constants.SwerveModuleConstants.LOCKED_WHEELS[0];
            desiredModuleStates[1] = Constants.SwerveModuleConstants.LOCKED_WHEELS[1];
            desiredModuleStates[2] = Constants.SwerveModuleConstants.LOCKED_WHEELS[2];
            desiredModuleStates[3] = Constants.SwerveModuleConstants.LOCKED_WHEELS[3];
        }

        this.frontLeftModule.setDesiredState(desiredModuleStates[0]);
        this.frontRightModule.setDesiredState(desiredModuleStates[1]);
        this.backLeftModule.setDesiredState(desiredModuleStates[2]);
        this.backRightModule.setDesiredState(desiredModuleStates[3]);
    }

    public ChassisSpeeds secondOrderKinematics (ChassisSpeeds chassisSpeeds) {

        Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        Translation2d rotationAdjustment = translation.rotateBy(new Rotation2d(-Math.PI / 2))
                .times(chassisSpeeds.omegaRadiansPerSecond * 0.045);

        translation = translation.plus(rotationAdjustment);
        return new ChassisSpeeds(translation.getX(), translation.getY(), chassisSpeeds.omegaRadiansPerSecond);
    }

    public void stop () {

        this.frontLeftModule.stop();
        this.frontRightModule.stop();
        this.backLeftModule.stop();
        this.backRightModule.stop();
    }

    public void setBrakeMode (boolean enable) {

        this.frontLeftModule.setBrake(enable);
        this.frontRightModule.setBrake(enable);
        this.backLeftModule.setBrake(enable);
        this.backRightModule.setBrake(enable);
    }

    public double getTilt () {
        return this.navX.getRoll();
    }

    public double getTiltVelocity () {
        return this.navX.getRawGyroY();
    }

    public void updateOdometry () {

        this.swerveDriveOdometry.update(Rotation2d.fromDegrees(this.navX.getYaw()), this.getModulePositions());
    }

    public void updateAutoOdometry () {

        this.autoOdometry.update(Rotation2d.fromDegrees(this.navX.getYaw()), this.getModulePositions());
    }

    public Rotation2d getGyro () {
        return Rotation2d.fromDegrees(this.navX.getYaw());
    }

    public Pose2d getPose () {

        Pose2d pose = this.swerveDriveOdometry.getPoseMeters();
        Translation2d position = pose.getTranslation();

        SmartDashboard.putNumber("Robot X", position.getX());
        SmartDashboard.putNumber("Robot Y", position.getY());
        SmartDashboard.putNumber("Robot Gyro", getGyro().getRadians());

        return pose;
    }

    public Pose2d getAutoPose () {

        this.updateAutoOdometry();
        return this.autoOdometry.getPoseMeters();
    }

    public Command toPose (Pose2d initialPose, Pose2d destinationPose, Supplier<Pose2d> currentPose) {

        ArrayList<PathPoint> points = new ArrayList<>();
        points.add(new PathPoint(initialPose.getTranslation(), initialPose.getRotation()));
        points.add(new PathPoint(destinationPose.getTranslation(), destinationPose.getRotation()));

        return new PPSwerveControllerCommand(
            PathPlanner.generatePath(
                new PathConstraints(
                    Constants.SwerveModuleConstants.TEST_MAX_VELOCITY,
                    Constants.SwerveModuleConstants.TEST_MAX_ACCELERATION
                ),
                points
            ),
            currentPose,
            new PIDController(0.0, 0.0, 0.0),
            new PIDController(0.0, 0.0, 0.0),
            new PIDController(0.0, 0.0, 0.0),
            this::setModuleStates
        );
    }

    public void resetOdometry (Pose2d pose) {

        this.navX.reset();
        this.navX.setAngleAdjustment(pose.getRotation().getDegrees());

        this.updateMaintainAngle();
        this.swerveDriveOdometry.resetPosition(Rotation2d.fromDegrees(this.navX.getYaw()).times(-1.0), this.getModulePositions(), pose);
        this.autoOdometry.resetPosition(Rotation2d.fromDegrees(this.navX.getYaw()).times(-1.0), this.getModulePositions(), pose);
    }

    public void setPose (Pose2d pose) {

        this.swerveDriveOdometry.resetPosition(Rotation2d.fromDegrees(this.navX.getYaw()).times(-1.0), this.getModulePositions(), pose);
    }

    public void resetOdometry (Rotation2d angle) {

        this.navX.reset();
        this.navX.setAngleAdjustment(angle.getDegrees());

        Pose2d pose = new Pose2d(getPose().getTranslation(), angle);
        this.updateMaintainAngle();

        this.swerveDriveOdometry.resetPosition(Rotation2d.fromDegrees(this.navX.getYaw()).times(-1.0), this.getModulePositions(), pose);
    }

    public ChassisSpeeds getChassisSpeed () {

        return Constants.SwerveModuleConstants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(
                this.frontLeftModule.getState(),
                this.frontRightModule.getState(),
                this.backLeftModule.getState(),
                this.backRightModule.getState());
    }

    public ChassisSpeeds getDesiredChassisSpeeds () {

        return Constants.SwerveModuleConstants.SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(
                this.desiredModuleStates[0],
                this.desiredModuleStates[1],
                this.desiredModuleStates[2],
                this.desiredModuleStates[3]);
    }

    public ChassisAccelerations getChassisAccelerations () {
        return this.driveAccelerations;
    }

    public SwerveModulePosition[] getModulePositions () {

        return new SwerveModulePosition[] {
                this.frontLeftModule.getPosition(),
                this.frontRightModule.getPosition(),
                this.backLeftModule.getPosition(),
                this.backRightModule.getPosition()
        };
    }

    private double performMaintainAngle (double xSpeed, double ySpeed, double rotation) {

        double output = rotation;

        if (Math.abs(rotation) >= Constants.ControllerConstants.MIN_ROTATION_COMMAND) {

            this.lastRotationTime = this.maintainAngleTimer.get();
        }

        if (Math.abs(xSpeed) >= Constants.ControllerConstants.MIN_TRANSLATION_COMMAND ||
                Math.abs(ySpeed) >= Constants.ControllerConstants.MIN_TRANSLATION_COMMAND) {

            this.lastDriveTime = this.maintainAngleTimer.get();
        }

        this.timeSinceRotation = this.maintainAngleTimer.get() - this.lastRotationTime;
        this.timeSinceDrive = this.maintainAngleTimer.get() - this.lastDriveTime;

        if (this.timeSinceRotation < 0.25) {
            this.maintainAngle = this.getGyro().getRadians();
        } else if (Math.abs(rotation) < Constants.ControllerConstants.MIN_ROTATION_COMMAND
                && this.timeSinceDrive < 0.25) {

            output = this.maintainAnglePID.calculate(this.getGyro().getRadians(), this.maintainAngle);
        }

        return output;
    }

    public void updateMaintainAngle () {
        this.maintainAngle = this.getGyro().getRadians();
    }

    public void changeSlewRate (double translation, double rotation) {

        this.slewRateLimiterX = new SlewRateLimiter(translation, this.latestSlewRates[0]);
        this.slewRateLimiterY = new SlewRateLimiter(translation, this.latestSlewRates[1]);
        this.slewRateLimiterZ = new SlewRateLimiter(rotation, this.latestSlewRates[2]);
    }
}
