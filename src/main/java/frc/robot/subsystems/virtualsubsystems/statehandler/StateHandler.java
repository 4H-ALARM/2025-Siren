// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.virtualsubsystems.statehandler;

import frc.lib.config.robotstateconfig;
import frc.lib.util.VirtualSubsystem;

public class StateHandler extends VirtualSubsystem {
  /** Creates a new StateHandler. */
  private robotstateconfig state;

  public StateHandler() {}

  public robotstateconfig getState() {
    return this.state;
  }

  @Override
  public void periodic() {}
}
