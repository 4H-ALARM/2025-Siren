package frc.lib.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.BasePosition;

public class RobotConstants {

  public static class DriveConstants {

    public static final double alignP = 1.85;
    public static final double alignI = 0.5;
    public static final double alignD = 0.1375;
    public static final double maxSpeed = 15.0;
    public static final double maxAccel = 30;
    public static final double translationRange = 0.02;

    public static final double headingP = 0.15; // 0.125 / 22;
    public static final double headingI = 0.03;
    public static final double headingD = 0;
    public static final double maxHeadingSpeed = 1.5;
    public static final double maxHeadingAccel = 3;
    public static final double headingRange = Units.degreesToRadians(1.5);
  }

  public static class ClimberConstants {

    public static final int climberMotorID = 50;
    public static final int limitSwitchID = 1;
  }

  public static class DealgifierConstants {

    public static final int leadID = 10;
    public static final int follower = 11;
    public static final double speed = 0.8;
    public static final double holdSpeed = 0.0;
  }

  public static class EndEffectorConstants {

    public static int wristmotorID = 32;
    public static int clawmotorID = 33;
    public static int frontcanrange = 40;
    public static int intakecanrange = 41;

    public static final Rotation2d defaultrot = new Rotation2d().fromRotations(0.50);
    public static final Rotation2d intakerot = new Rotation2d().fromRotations(0.8);
    public static final Rotation2d dealgifyrot = new Rotation2d().fromRotations(0.75);
    public static final Rotation2d holdAlgaerot = new Rotation2d().fromRotations(0.77);
    public static final Rotation2d L1rot = new Rotation2d().fromRotations(0.72);
    public static final Rotation2d L2rot = new Rotation2d().fromRotations(0.72);
    public static final Rotation2d L3rot = new Rotation2d().fromRotations(0.73);
    public static final Rotation2d L4rot = new Rotation2d().fromRotations(0.71);

    public static final double staticSpeed = 0.0;
    public static final double intakeSpeed = -0.8;
    public static final double placeSpeed = -1;
    public static final double centerForwardSpeed = -0.3;
    public static final double centerBackwardsSpeed = 0.3;
  }

  public static class ElevatorConstants {
    public static final int elevatorLeft = 30;
    public static final int elevatorRight = 31;
    public static final int bottomlimitswitchID = 3;
    public static final int toplimitswitchID = 1;
    public static final int candi = 60;

    public static final BasePosition CORAL_L1 = new BasePosition(0.0);

    public static final BasePosition CORAL_L2 = new BasePosition(0.19);
    public static final BasePosition CORAL_L3 = new BasePosition(0.47);
    public static final BasePosition CORAL_L4 = new BasePosition(0.985);
    public static final BasePosition BOTTOM = new BasePosition(0.0);
    public static final BasePosition DEALGIFYLOW = new BasePosition(0.235 / 2);
    public static final BasePosition DEALGIFYHIGH = new BasePosition(0.54);
    public static final double encoderLowerLimit = 0.0;
    public static final double encoderUpperLimit = 23.15;
    public static final double closeEnoughRange = 0.01 * ElevatorConstants.encoderUpperLimit;
  }

  public static class GroundIntakeConstants {

    public static final int tiltMotorID = 51;
    public static final int spinMotorID = 34;

    public static final double staticSpeed = 0.0;
    public static final double intakeSpeed = 0.8;
    public static final double throwSpeed = -1;
    public static final double holdSpeed = 0.03;

    public static final Rotation2d defaultangle = Rotation2d.fromRotations(0);
    public static final Rotation2d intakingangle = Rotation2d.fromRotations(-10);
    public static final Rotation2d holdangle = Rotation2d.fromRotations(0);
  }

  public static class GeneralConstants {

    public static Pose2d[] getCartesianCoordinates(
        double angle, double poseOffset, double poseOffsetBack) { // Left:trueRight:falss

      double radians = Units.degreesToRadians(angle);
      // Calculate x and y using trigonometric functions
      double x = (0.8315 + poseOffsetBack) * Math.cos(radians) + 4.4895;
      double y = (0.8315 + poseOffsetBack) * Math.sin(radians) + 4.026;

      // Compute the tangent line's direction
      double tangentX = -Math.sin(radians); // Negative sine for perpendicular direction
      double tangentY = Math.cos(radians); // Cosine for perpendicular direction

      // Normalize the tangent direction
      double magnitude = Math.sqrt(tangentX * tangentX + tangentY * tangentY);
      tangentX /= magnitude;
      tangentY /= magnitude;

      // Calculate pose1 and pose2 positions along the tangent line
      double x1 = x + tangentX * poseOffset;
      double y1 = y + tangentY * poseOffset;

      double x2 = x - tangentX * poseOffset;
      double y2 = y - tangentY * poseOffset;

      // Create Pose2d objects
      Pose2d pose1 = new Pose2d(new Translation2d(x1, y1), Rotation2d.fromRadians(radians));
      Pose2d pose2 = new Pose2d(new Translation2d(x2, y2), Rotation2d.fromRadians(radians));

      // Return the coordinates as a Pose2d array
      return new Pose2d[] {pose1, pose2};
    }

    public static boolean DEBUG = true;

    public static Pose2d[] zerodeg = getCartesianCoordinates(0, 0.164, 0.48);
    public static Pose2d[] sixtydeg = getCartesianCoordinates(60, 0.164, 0.48);
    public static Pose2d[] onetwentydeg = getCartesianCoordinates(120, 0.164, 0.48);
    public static Pose2d[] oneeightydeg = getCartesianCoordinates(180, 0.164, 0.48);
    public static Pose2d[] twofourtydeg = getCartesianCoordinates(240, 0.164, 0.48);
    public static Pose2d[] threehundreddeg = getCartesianCoordinates(300, 0.164, 0.48);

    public static Pose2d[] reefPoses = {
      zerodeg[1],
      zerodeg[0],
      sixtydeg[1],
      sixtydeg[0],
      onetwentydeg[1],
      onetwentydeg[0],
      oneeightydeg[1],
      oneeightydeg[0],
      twofourtydeg[1],
      twofourtydeg[0],
      threehundreddeg[1],
      threehundreddeg[0]
    };

    public static Pose2d[] zerodegA = getCartesianCoordinates(0, 0, 0.44);
    public static Pose2d[] sixtydegA = getCartesianCoordinates(60, 0, 0.44);
    public static Pose2d[] onetwentydegA = getCartesianCoordinates(120, 0, 0.44);
    public static Pose2d[] oneeightydegA = getCartesianCoordinates(180, 0, 0.44);
    public static Pose2d[] twofourtydegA = getCartesianCoordinates(240, 0, 0.44);
    public static Pose2d[] threehundreddegA = getCartesianCoordinates(300, 0, 0.44);

    public static Pose2d[] algaePoses = {
      zerodegA[0],
      sixtydegA[0],
      onetwentydegA[0],
      oneeightydegA[0],
      twofourtydegA[0],
      threehundreddegA[0]
    };

    public static Pose2d[] intakePoses = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};
  }
}
