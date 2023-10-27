package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.Constants;

public class MathUtils {
    
    private static final double EPS = 1E-9;
    
    public static double toUnitCircAngle (double angle) {

        double rotations = angle / (2 * Math.PI);
        return (angle - Math.round(rotations - 0.500) * Math.PI * 2.0);
    }

    public static double singedSquare (double input) {

        return Math.signum(input) * Math.pow(input, 2);
    }

    public static double cubicLinear (double input, double a, double b) {

        return (a * Math.pow(input, 3)+ b * input);
    }

    public static double applyDeadband (double input) {

        if (Math.abs(input) < Constants.ControllerConstants.INNER_DEADBAND) { return 0.0; } 
        else if (Math.abs(input) > Constants.ControllerConstants.OUTER_DEADBAND) { return Math.signum(input) * 1.0; }
        return input;
    }

    public static double inputTransform (double input) {

      //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
      return cubicLinear(applyDeadband(input), 0.95, 0.05);
    }

    public static double[] inputTransform (double x, double y) {

        x = applyDeadband(x);
        y = applyDeadband(y);
        double magnitude = pythagorean(x,y);
        if (magnitude > 1.00) { magnitude = 1.00; }

        if (magnitude != 0.00) {

            x = x / magnitude * cubicLinear(magnitude, 0.95, 0.05);
            y = y / magnitude * cubicLinear(magnitude, 0.95, 0.05);
        } else {

            x = 0.00;
            y = 0.00;
        }

        return new double[]{x,y};
    }

    public static double pythagorean (double a, double b) {

      return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    }
    
    public static Twist2d log (final Pose2d transform) {

        final double deltaTheta = transform.getRotation().getRadians();
        final double halfDeltaTheta = 0.5 * deltaTheta;
        final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
        double halfDeltaThetaByTanOfHalfDeltaTheta;

        if (Math.abs(cos_minus_one) < EPS) {

            halfDeltaThetaByTanOfHalfDeltaTheta = 1.0 - 1.0 / 12.0 * deltaTheta * deltaTheta;
        } else {

            halfDeltaThetaByTanOfHalfDeltaTheta = -(halfDeltaTheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
        }

        final Translation2d translationPart = transform.getTranslation().rotateBy(new Rotation2d(halfDeltaThetaByTanOfHalfDeltaTheta, -halfDeltaTheta));
        return new Twist2d(translationPart.getX(), translationPart.getY(), deltaTheta);
  }

    public static double normalizeAngle (double angle) {

      return angle - (2 * Math.PI * Math.floor(angle / (2 * Math.PI)));
    }

    public static double normalizeAngle (double angle, double reference) {

      return angle + (2 * Math.PI * Math.floor(reference / (2 * Math.PI)));
    }

    // TODO
    public static Translation2d getIntersection () {

      return new Translation2d();
    }
}
