// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bargemech;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

public class bargeMech extends SubsystemBase {

  private final bargeIO barge;
  private final StateHandler stateHandler;

  /** Creates a new EndEffector. */
  public bargeMech(bargeIO b, StateHandler handler) {
    this.barge = b;
    stateHandler = handler;
  }

  public void move(double input) {
    this.barge.intake(input);
  }

  public void stopBarge() {
    this.barge.stopMotor();
  }

  @Override
  public void periodic() {
    this.barge.intake(this.stateHandler.getState().getDealgifySpeed());
  }
}
