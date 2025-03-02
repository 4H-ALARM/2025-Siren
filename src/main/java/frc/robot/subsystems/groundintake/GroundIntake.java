// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {

  private IntakeIO intake;

  /** Creates a new GroundIntake. */
  public GroundIntake() {}

  public void setAngle(Rotation2d angle) {
    this.intake.setAngle(angle);
  }

  public void stopMotor() {
    this.intake.setSpeed(0);
    ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
