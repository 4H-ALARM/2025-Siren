package frc.lib.constants;

import edu.wpi.first.math.util.Units;

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

  public static class GeneralConstants {}
}
