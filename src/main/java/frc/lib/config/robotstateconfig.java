// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.constants.RobotConstants;
import frc.lib.enums.TargetPose;

/** Add your docs here. */
public class robotstateconfig {

  private Rotation2d wristRotation;
  private Rotation2d groundIntakeRot;
  private double groundIntakeSpeed;
  private double clawSpeed;
  public double daSpeed;
  private TargetPose targetPose;
  private boolean disabled;

  public robotstateconfig(
      Rotation2d wristRot,
      Rotation2d groundIntakeRot,
      double groundIntakeSpeed,
      double clawSpeed,
      double daSpeed,
      TargetPose targetPose,
      boolean disabled) {
    this.wristRotation = wristRot;
    this.groundIntakeRot = groundIntakeRot;
    this.groundIntakeSpeed = groundIntakeSpeed;
    this.clawSpeed = clawSpeed;
    this.daSpeed = daSpeed;
    this.targetPose = targetPose;
    this.disabled = disabled;
  }

  public robotstateconfig() {
    this(
        RobotConstants.EndEffectorConstants.defaultrot,
        RobotConstants.GroundIntakeConstants.defaultangle,
        0.0,
        0.0,
        0.0,
        TargetPose.NONE,
        true);
  }

  public robotstateconfig(
      Rotation2d wristRot,
      Rotation2d groundIntakeRot,
      double groundIntakeSpeed,
      double clawSpeed,
      double daSpeed,
      TargetPose targetPose) {
    this(wristRot, groundIntakeRot, groundIntakeSpeed, clawSpeed, daSpeed, targetPose, false);
  }

  public Rotation2d getWristRotation() {
    return wristRotation;
  }

  public void setWristRotation(Rotation2d wristRotation) {
    this.wristRotation = wristRotation;
  }

  public Rotation2d getGroundIntakeRot() {
    return groundIntakeRot;
  }

  public void setGroundIntakeRot(Rotation2d groundIntakeRot) {
    this.groundIntakeRot = groundIntakeRot;
  }

  public double getGroundIntakeSpeed() {
    return groundIntakeSpeed;
  }

  public void setDealgifySpeed(double daSpeed) {
    this.daSpeed = daSpeed;
  }

  public double getDealgifySpeed() {
    return this.daSpeed;
  }

  public void setGroundIntakeSpeed(double groundIntakeSpeed) {
    this.groundIntakeSpeed = groundIntakeSpeed;
  }

  public double getClawSpeed() {
    return clawSpeed;
  }

  public void setClawSpeed(double clawSpeed) {
    this.clawSpeed = clawSpeed;
  }

  public TargetPose getTargetPose() {
    return targetPose;
  }

  public void setTargetPose(TargetPose targetPose) {
    this.targetPose = targetPose;
  }

  public boolean isDisabled() {
    return disabled;
  }

  public void setDisabled(boolean disabled) {
    this.disabled = disabled;
  }
}
