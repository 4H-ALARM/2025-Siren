// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.constants.RobotConstants;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    Rotation2d axleRotation;
    double height;
    boolean leadisConnected;
    boolean followerisConnected;
    String enumState;
  }

  public void moveToState(RobotConstants.ElevatorConstants.elevatorState state);

  public void move(double input);

  public void stopElevator();

  public RelativeEncoder getEncoder();

  public void resetEncoder();

  public void moveToPoint(Rotation2d targetRot);

  public double getPercentRaised();

  public default void updateInputs(ElevatorIOInputs inputs) {}
}
