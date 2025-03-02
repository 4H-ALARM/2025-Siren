// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class IntakeConstants {

  public static final int tiltMotorID = 1;
  public static final int spinMotorID = 2;

  public static final Rotation2d inactiveAngle = Rotation2d.fromDegrees(0);
  public static final Rotation2d activeAngle = Rotation2d.fromDegrees(0);
  public static final Rotation2d holdAngle = Rotation2d.fromDegrees(0);
}
