package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;
    private final WPI_CANCoder absoluteEncoder;
  
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;
    private double referenceAngle = 0.0;

    private final PIDController drivePIDController;
    private SimpleMotorFeedforward driveFeedForward;
  
    private final PIDController steerPIDController = new PIDController(
        Constants.SwerveModuleConstants.STEER_PID[0],
        Constants.SwerveModuleConstants.STEER_PID[1],
        Constants.SwerveModuleConstants.STEER_PID[2]
    );
  
    private final SparkMaxPIDController motorSteeringPID;
    private final SparkMaxPIDController motorDrivingPID;

    public SwerveModule (int driveMotor, int steerMotor, int absoluteEncoder, double encoderOffset) {
  
        this.driveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);
        this.driveMotor.setSmartCurrentLimit(57);
        this.driveMotor.enableVoltageCompensation(12.6);
        this.driveMotor.setInverted(false);
        this.driveMotor.setIdleMode(IdleMode.kBrake);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.driveEncoder.setVelocityConversionFactor(Constants.SwerveModuleConstants.VELOCITY_FACTOR);
        this.driveEncoder.setPositionConversionFactor(Constants.SwerveModuleConstants.VELOCITY_FACTOR * 60.0);
        this.driveEncoder.setAverageDepth(4);
        this.driveEncoder.setMeasurementPeriod(16);

        this.motorDrivingPID = this.driveMotor.getPIDController();
        this.motorDrivingPID.setP(Constants.SwerveModuleConstants.DRIVE_CONTROLLER[2]);

        this.driveMotor.burnFlash();

        this.steerMotor = new CANSparkMax(steerMotor, MotorType.kBrushless);
        this.steerMotor.setSmartCurrentLimit(20);
        this.steerMotor.enableVoltageCompensation(12.6);
        this.steerMotor.setInverted(false);
        this.steerMotor.setIdleMode(IdleMode.kBrake);

        this.steerEncoder = this.steerMotor.getEncoder();
        this.steerEncoder.setVelocityConversionFactor(Constants.SwerveModuleConstants.MODULE_CONFIGURATION.getDriveReduction() * 2 * Math.PI / 60.0);
        this.steerEncoder.setPositionConversionFactor(Constants.SwerveModuleConstants.MODULE_CONFIGURATION.getDriveReduction() * 2 * Math.PI);
        this.steerEncoder.setAverageDepth(4);

        this.motorSteeringPID = this.steerMotor.getPIDController();
        this.motorSteeringPID.setP(Constants.SwerveModuleConstants.NEO_STEER_P);

        this.steerMotor.burnFlash();

        this.absoluteEncoder = new WPI_CANCoder(absoluteEncoder);
        this.absoluteEncoder.configMagnetOffset(encoderOffset);
        this.absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        this.driveFeedForward = new SimpleMotorFeedforward(
            Constants.SwerveModuleConstants.DRIVE_CONTROLLER[0],
            Constants.SwerveModuleConstants.DRIVE_CONTROLLER[1]
        );

        this.drivePIDController = new PIDController(Constants.SwerveModuleConstants.DRIVE_CONTROLLER[2], 0.0, 0.0);
        this.steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState () {

        return new SwerveModuleState(this.driveEncoder.getVelocity(), new Rotation2d(this.getTurnEncoder()));
    }

    public SwerveModulePosition getPosition () {

        return new SwerveModulePosition(this.driveEncoder.getPosition(), new Rotation2d(this.getTurnEncoder()));
    }
  
    public void setDesiredState (SwerveModuleState desiredState) {
        
      SwerveModuleState swerveModuleState = SwerveModuleState.optimize(desiredState, new Rotation2d(this.getTurnEncoder()));

      //final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);
      final double driveFF = this.driveFeedForward.calculate(swerveModuleState.speedMetersPerSecond);
      this.motorDrivingPID.setReference(swerveModuleState.speedMetersPerSecond, ControlType.kVelocity, 0, driveFF * 12.6);
      
      final double steerOutput = this.steerPIDController.calculate(this.getTurnEncoder(), swerveModuleState.angle.getRadians());
      this.steerMotor.set(steerOutput);
    }
  
    public void setBrake (boolean brake) {

        if (brake) { this.driveMotor.setIdleMode(IdleMode.kBrake); }
        else { this.driveMotor.setIdleMode(IdleMode.kCoast); }
    }
  
    public void stop () {

        this.driveMotor.set(0.0);
        this.steerMotor.set(0.0);
    }
  
    public double getTurnEncoder () {

        return -1.0 * this.absoluteEncoder.getAbsolutePosition() / 360.0 * 2 * Math.PI;
    }
  
    public void setReferenceAngle (double referenceAngle) {

        double currentAngle = this.steerEncoder.getPosition();
        double currentAngleMod = currentAngle % (2 * Math.PI);
        if (currentAngleMod < 0.0) { currentAngleMod += 2.0 * Math.PI; }
  
        double adjustedReferenceAngle = referenceAngle + currentAngle - currentAngleMod;

        if (referenceAngle - currentAngleMod > Math.PI) { adjustedReferenceAngle -= 2.0 * Math.PI; } 
        else if (referenceAngle - currentAngleMod < -Math.PI) { adjustedReferenceAngle += 2.0 * Math.PI; }
  
        this.referenceAngle = referenceAngle;
        this.motorSteeringPID.setReference(adjustedReferenceAngle, ControlType.kPosition);
    }
  
    public double getReferenceAngle () { return this.referenceAngle; }
  
    public double getStateAngle () {

        double motorAngle = this.steerEncoder.getPosition();
        motorAngle %= 2.0 * Math.PI;

        if (motorAngle < 0.0) { motorAngle += 2.0 * Math.PI; }
        return motorAngle;
    }
}
