// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.enums;

import frc.lib.config.robotstateconfig;
import frc.lib.constants.RobotConstants.DealgifierConstants;
import frc.lib.constants.RobotConstants.EndEffectorConstants;
import frc.lib.constants.RobotConstants.GroundIntakeConstants;

/** Add your docs here. */
public enum robotStates {
  STOP(
      new robotstateconfig(
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE,
          true)),
  RESTING(
      new robotstateconfig(
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE)),

  RESTINGPOSTDEALGIFY(
      new robotstateconfig(
          EndEffectorConstants.holdAlgaerot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          DealgifierConstants.holdSpeed,
          TargetPose.NONE)),

  RESTINGPOSTDEALGIFYNOHOLD(
      new robotstateconfig(
          EndEffectorConstants.holdAlgaerot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE)),
  INTAKE(
      new robotstateconfig(
          EndEffectorConstants.intakerot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.intakeSpeed,
          0,
          TargetPose.NONE,
          false)),
  INTAKECENTERFORWARD(
      new robotstateconfig(
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.centerForwardSpeed,
          0,
          TargetPose.NONE)),
  INTAKECENTERBACKWARD(
      new robotstateconfig(
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.centerBackwardsSpeed,
          0,
          TargetPose.NONE)),
  L1PREPARE(
      new robotstateconfig(
          EndEffectorConstants.L1rot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE)),
  L1SCORE(
      new robotstateconfig(
          EndEffectorConstants.L1rot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.placeSpeed,
          0,
          TargetPose.NONE)),
  L2PREPARE(
      new robotstateconfig(
          EndEffectorConstants.L2rot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE)),
  L2SCORE(
      new robotstateconfig(
          EndEffectorConstants.L2rot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.placeSpeed,
          0,
          TargetPose.NONE)),
  L3PREPARE(
      new robotstateconfig(
          EndEffectorConstants.L3rot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE)),
  L3SCORE(
      new robotstateconfig(
          EndEffectorConstants.L3rot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.placeSpeed,
          0,
          TargetPose.NONE)),
  L4PREPARE(
      new robotstateconfig(
          EndEffectorConstants.L4rot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE)),
  L4SCORE(
      new robotstateconfig(
          EndEffectorConstants.L4rot,
          GroundIntakeConstants.defaultangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.placeSpeed,
          0,
          TargetPose.NONE)),
  GROUNDINTAKE(
      new robotstateconfig(
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.intakingangle,
          GroundIntakeConstants.intakeSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE)),
  GROUNDHOLD(
      new robotstateconfig(
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.holdangle,
          GroundIntakeConstants.holdSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE)),
  GROUNDTHROW(
      new robotstateconfig(
          EndEffectorConstants.defaultrot,
          GroundIntakeConstants.holdangle,
          GroundIntakeConstants.throwSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE)),

  DEALGIFYLOWPREPARE(
      new robotstateconfig(
          EndEffectorConstants.dealgifyrot,
          GroundIntakeConstants.holdangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE)),

  DEALGIFYHIGHPREPARE(
      new robotstateconfig(
          EndEffectorConstants.dealgifyrot,
          GroundIntakeConstants.holdangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          0,
          TargetPose.NONE)),

  DEALGIFYLOW(
      new robotstateconfig(
          EndEffectorConstants.dealgifyrot,
          GroundIntakeConstants.holdangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          DealgifierConstants.speed,
          TargetPose.NONE)),

  DEALGIFYHIGH(
      new robotstateconfig(
          EndEffectorConstants.dealgifyrot,
          GroundIntakeConstants.holdangle,
          GroundIntakeConstants.staticSpeed,
          EndEffectorConstants.staticSpeed,
          DealgifierConstants.speed,
          TargetPose.NONE));

  private robotstateconfig targetrobotstate;

  public robotstateconfig getRobotStateConfig() {
    return targetrobotstate;
  }

  private robotStates(robotstateconfig targetstate) {
    this.targetrobotstate = targetstate;
  }
}
