// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final XboxController driverOneController = new XboxController(0);
  private final AHRS navX = new AHRS(Port.kMXP);
  private final Limelight limelight = new Limelight("limelight");

  private final Field2d field2d = new Field2d();
  private final SwerveDrive swerveDrive = new SwerveDrive(navX, field2d);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private File[] autoPathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/").listFiles();
  private final HashMap<String, Command> autoEvents = new HashMap<>();

  private final PoseEstimator poseEstimator = new PoseEstimator(swerveDrive, limelight, new Pose2d());

  private final SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
    poseEstimator::getPose,
    poseEstimator::resetOdometry, 
    new PIDConstants(5.0, 0, 0), 
    new PIDConstants(5.0,0.0,0), 
    swerveDrive::setModuleStates, 
    autoEvents, 
    true, 
    swerveDrive,
    limelight
  );

  public RobotContainer() {

    this.configureButtonBindings();
    this.configureDashboard();

    this.swerveDrive.setDefaultCommand(new Drive(this.swerveDrive, this.driverOneController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings () {

    new POVButton(this.driverOneController, 0).onTrue(new InstantCommand(() -> this.swerveDrive.resetOdometry(new Pose2d())));
  }

  private void configureDashboard () {

    this.autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1.0));

    for (File auto : this.autoPathFiles) {
      
      if (auto.getName().contains(".path")) {

        this.autoChooser.addOption(
          auto.getName(), 
          this.swerveAutoBuilder.fullAuto(PathPlanner.loadPathGroup(auto.getName().replace(".path", ""), 4.0, 2.75))
        );
      }
    }

    Shuffleboard.getTab("Autonomous").addPersistent("Autonomous Selector", this.autoChooser).withWidget("ComboBox Chooser");
    Shuffleboard.getTab("Autonomous").add(this.field2d).withWidget("Field").withProperties(Map.of("robot_width", 0.83566, "robot_length", 0.83566));

    Shuffleboard.getTab("Teleoperated").add("NavX", this.navX).withWidget("Gyro");
    Shuffleboard.getTab("Teleoperated").add("Swerve Drive", new Sendable() {
        
        @Override
        public void initSendable (SendableBuilder sendableBuilder) {

            sendableBuilder.setSmartDashboardType("SwerveDrive");
            SwerveModuleState[] swerveModuleStates = swerveDrive.getModuleStates();

            sendableBuilder.addDoubleProperty("Front Left Angle", () -> swerveModuleStates[0].angle.getDegrees(), null);
            sendableBuilder.addDoubleProperty("Front Left Velocity", () -> swerveModuleStates[0].speedMetersPerSecond, null);

            sendableBuilder.addDoubleProperty("Front Right Angle", () -> swerveModuleStates[1].angle.getDegrees(), null);
            sendableBuilder.addDoubleProperty("Front Right Velocity", () -> swerveModuleStates[1].speedMetersPerSecond, null);

            sendableBuilder.addDoubleProperty("Back Left Angle", () -> swerveModuleStates[2].angle.getDegrees(), null);
            sendableBuilder.addDoubleProperty("Back Left Velocity", () -> swerveModuleStates[2].speedMetersPerSecond, null);

            sendableBuilder.addDoubleProperty("Back Right Angle", () -> swerveModuleStates[3].angle.getDegrees(), null);
            sendableBuilder.addDoubleProperty("Back Right Velocity", () -> swerveModuleStates[3].speedMetersPerSecond, null);

            sendableBuilder.addDoubleProperty("Robot Angle", () -> navX.getRotation2d().getDegrees(), null);
        }
    });
  }

  public Command getAutonomousCommand() { return this.autoChooser.getSelected(); }
  public void updateTrajectory () { 
    
    Trajectory trajectory = PathPlanner.loadPath(this.autoChooser.getSelected().getName(), 4.0, 2.75);
    this.field2d.getObject("Trajectory").setTrajectory(trajectory);
  }
}
