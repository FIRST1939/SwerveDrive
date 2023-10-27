// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final SwerveDrive swerveDrive = new SwerveDrive(navX);

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
    this.configureAutoChooser();

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

  private void configureAutoChooser () {

    this.autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1.0));

    for (File auto : this.autoPathFiles) {
      
      if (auto.getName().contains(".path")) {

        this.autoChooser.addOption(
          auto.getName(), 
          this.swerveAutoBuilder.fullAuto(PathPlanner.loadPathGroup(auto.getName().replace(".path", ""), 4.0, 2.75))
        );
      }
    }

    SmartDashboard.putData(this.autoChooser);
  }

  public Command getAutonomousCommand() { return this.autoChooser.getSelected(); }
}
