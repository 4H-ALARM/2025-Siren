// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {
  private final GroundIntakeIO gi;
  private final GroundIntakeIOInputsAutoLogged inputs;
  /** Creates a new GroundIntake. */
  public GroundIntake(GroundIntakeIO groundintakeimpl) {
    this.gi = groundintakeimpl;
    inputs = new GroundIntakeIOInputsAutoLogged();
  }

  public void moveAngle(double input) {
    this.gi.moveAngle(input);
  }

  public void setAngle(double input) {
    this.gi.setAngle(Rotation2d.fromRotations(input));
  }

  public void setSpeed(double speed) {
    this.gi.setSpeed(speed);
  }

  public void resetEncoder() {
    this.gi.resetEncoder();
  }

  public void stopMotors() {
    this.gi.stopMotors();
  }

  @Override
  public void periodic() {
    this.gi.updateInputs(inputs);
  }
}
