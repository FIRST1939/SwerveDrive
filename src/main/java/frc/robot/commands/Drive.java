package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.MathUtils;

public class Drive extends CommandBase {
    
    private final SwerveDrive swerveDrive;
    private final XboxController xboxController;

    private boolean fieldOrient = true;

    public Drive (SwerveDrive swerveDrive, XboxController xboxController) {

        this.swerveDrive = swerveDrive;
        this.xboxController = xboxController;
        this.addRequirements(this.swerveDrive);
    }

    @Override
    public void execute () {

        if (!this.xboxController.getRightBumper()) {

            double desiredTranslations[] = MathUtils.inputTransform(-xboxController.getLeftY(), this.xboxController.getLeftX());
            double maxLinear = Constants.SwerveModuleConstants.MAX_VELOCITY;
            desiredTranslations[0] *= maxLinear;
            desiredTranslations[1] *= maxLinear;

            double desiredRotation = -MathUtils.inputTransform(-this.xboxController.getRightX()) * Constants.SwerveModuleConstants.MAX_ANGULAR_VELOCITY;
            desiredTranslations[0] *= (1 - this.xboxController.getLeftTriggerAxis());
            desiredTranslations[1] *= (1 - this.xboxController.getLeftTriggerAxis());

            this.swerveDrive.drive(
                desiredTranslations[0],
                desiredTranslations[1],
                desiredRotation,
                this.fieldOrient,
                true
            );
        } else {

            this.swerveDrive.setModuleStates(Constants.SwerveModuleConstants.LOCKED_WHEELS);
        }
    }

    @Override
    public void end (boolean interrupted) {}
}
