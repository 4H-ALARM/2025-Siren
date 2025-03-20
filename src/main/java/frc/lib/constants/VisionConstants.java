// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.lib.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  // Camera names, must match names configured on coprocessor
  public static String camera1Name = "alarmcamera1";
  public static String camera2Name = "alarmcamera2";
  public static String camera3Name = "alarmcamera3";
  public static String camera4Name = "alarmcamera4";
  public static String limelightName = "limelight-intake";

  private static final double xMeters = Units.inchesToMeters(11.375);
  private static final double yMeters = Units.inchesToMeters(11.375);
  private static final double zMeters = Units.inchesToMeters(7.906);

  private static final double rollRadians = Units.degreesToRadians(0);
  private static final double pitchRadians = Units.degreesToRadians(-15);
  private static final double yawRadians = Units.degreesToRadians(45);

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)

  // back left camera
  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(-11.317),
          Units.inchesToMeters(-11.292),
          zMeters + Units.inchesToMeters(0.5), // TODO
          new Rotation3d(rollRadians, Units.degreesToRadians(-15), Units.degreesToRadians(150)));

  // front right camera
  public static Transform3d robotToCamera2 =
      new Transform3d(
          Units.inchesToMeters(-11.317),
          Units.inchesToMeters(11.292),
          zMeters + Units.inchesToMeters(0.5), // TODO
          new Rotation3d(rollRadians, Units.degreesToRadians(-15), Units.degreesToRadians(210)));
  // rear left camera
  public static Transform3d robotToCamera3 =
      new Transform3d(
          -xMeters,
          yMeters,
          zMeters + Units.inchesToMeters(2), // TODO
          new Rotation3d(rollRadians, pitchRadians, Units.degreesToRadians(135)));
  // rear right camera
  public static Transform3d robotToCamera4 =
      new Transform3d(
          -xMeters,
          -yMeters,
          zMeters + Units.inchesToMeters(2), // TODO
          new Rotation3d(rollRadians, pitchRadians, Units.degreesToRadians(-135)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors = new double[] {0.3, 0.3};

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
