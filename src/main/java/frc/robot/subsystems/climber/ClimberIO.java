// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    double angle = 0;
    ;
  }

  public void move(double movement);

  public void stopMotor();

  public default void updateInputs(ClimberIOInputs inputs) {}
  ;
}
