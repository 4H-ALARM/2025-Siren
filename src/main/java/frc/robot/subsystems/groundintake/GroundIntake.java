// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

public class GroundIntake extends SubsystemBase {
  private final StateHandler stateHandler;
  private final GroundIntakeIO gi;
  private final GroundIntakeIOInputsAutoLogged inputs;
  /** Creates a new GroundIntake. */
  public GroundIntake(GroundIntakeIO groundintakeimpl, StateHandler handler) {
    this.stateHandler = handler;
    this.gi = groundintakeimpl;
    this.inputs = new GroundIntakeIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    this.gi.setAngle(stateHandler.getState().getGroundIntakeRot());
    this.gi.setSpeed(stateHandler.getState().getGroundIntakeSpeed());
    this.gi.updateInputs(inputs);
  }
}
