// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.BasePosition;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    Rotation2d axleRotation;
    Rotation2d motorRotation;
    double error;
    double height;
    boolean leadisConnected;
    boolean followerisConnected;
  }

  public void periodic();

  public void move(double input);

  public void stopElevator();

  public double getEncoder();

  public void resetEncoder();

  public void setTargetPosition(BasePosition position);

  public BasePosition getBasePosition();

  public default void updateInputs(ElevatorIOInputs inputs) {}
}
