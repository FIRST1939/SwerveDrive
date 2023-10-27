// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public final class Constants {

    // Controller Constants.
    public static final class ControllerConstants {

        public static final int LEFT_JOYSTICK = 0;
        public static final int RIGHT_JOYSTICK = 1;

        public static final double INNER_DEADBAND = 0.075;
        public static final double OUTER_DEADBAND = 0.975;

        public static final double MIN_ROTATION_COMMAND = Constants.SwerveModuleConstants.MAX_ANGULAR_VELOCITY * Math.pow(INNER_DEADBAND, 2);
        public static final double MIN_TRANSLATION_COMMAND = Constants.SwerveModuleConstants.MAX_VELOCITY * Math.pow(INNER_DEADBAND, 2);
    }

    // Swerve Constants.
    public static final class SwerveModuleConstants {

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13;
        public static final double FRONT_LEFT_MODULE_ENCODER_OFFSET = -38.6;

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 14;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 15;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 16;
        public static final double FRONT_RIGHT_MODULE_ENCODER_OFFSET = 158.6;

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 17;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 18;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 19;
        public static final double BACK_LEFT_MODULE_ENCODER_OFFSET = -34.9;

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 20;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 21;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22;
        public static final double BACK_RIGHT_MODULE_ENCODER_OFFSET = -251.1;
        
        public static final ModuleConfiguration MODULE_CONFIGURATION = SdsModuleConfigurations.MK4I_L3;
        public static final double VELOCITY_FACTOR = MODULE_CONFIGURATION.getDriveReduction() / 60.0 * MODULE_CONFIGURATION.getWheelDiameter() * Math.PI;

        public static final double WHEELBASE_WIDTH = 0.5588;
        public static final double WHEELBASE_LENGTH = 0.5588;

        public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2),
            new Translation2d(WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2),
            new Translation2d(-WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2),
            new Translation2d(-WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2)
        );

        public static final double MAX_VELOCITY = 4.5;
        public static final double MAX_ACCELERATION = 3.0;

        public static final double TEST_MAX_VELOCITY = 1.0;
        public static final double TEST_MAX_ACCELERATION = 1.0;

        public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION = 1.5 * Math.PI;

        // Static Gain, Feedforward, Proportionality
        public static final double[] DRIVE_CONTROLLER = { 0.015, 0.19, 0.15 };

        public static final double NEO_STEER_P = 3.0;
        public static final double[] STEER_PID = { 0.800, 0.0, 0.0 };
        public static final double[] MAINTAIN_ANGLE_PID = { 0.500, 0.0, 0.0 };

        public static final SwerveModuleState[] LOCKED_WHEELS_HELPER = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));
        public static final SwerveModuleState[] LOCKED_WHEELS = {
            new SwerveModuleState(0.0, LOCKED_WHEELS_HELPER[0].angle.rotateBy(new Rotation2d(-Math.PI * 1 / 4))),
            new SwerveModuleState(0.0, LOCKED_WHEELS_HELPER[1].angle.rotateBy(new Rotation2d(Math.PI * 1 / 4))),
            new SwerveModuleState(0.0, LOCKED_WHEELS_HELPER[2].angle.rotateBy(new Rotation2d(Math.PI * 1 / 4))),
            new SwerveModuleState(0.0, LOCKED_WHEELS_HELPER[3].angle.rotateBy(new Rotation2d(-Math.PI * 1 / 4)))
        };
    }

    // Limelight Constants.
    public static final class LimelightConstants {

        public static final double kPoseErrorAcceptance = 2.0;
    }
}
