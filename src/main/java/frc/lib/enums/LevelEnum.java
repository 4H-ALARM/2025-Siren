// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.enums;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.constants.RobotConstants;

/** Add your docs here. */
public enum LevelEnum {
  L1(RobotConstants.ElevatorConstants.L1height),
  L2(RobotConstants.ElevatorConstants.L2height),
  L3(RobotConstants.ElevatorConstants.L3height),
  L4(RobotConstants.ElevatorConstants.L4height);

  private final Rotation2d target;

  LevelEnum(Rotation2d rotation2d) {
    this.target = rotation2d;
  }

  public Rotation2d getTargetRotation2d() {
    return target;
  }
}
