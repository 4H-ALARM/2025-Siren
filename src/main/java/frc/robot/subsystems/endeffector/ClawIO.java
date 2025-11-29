package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface ClawIO {
  @AutoLog
  public static class ClawIOInputs {}

  public void setSpeed(double speed);

  public double getCurrentSpeed();

  public double getPosition();

  public void stopMotor();

  public boolean getFrontIntaked();

  public boolean getIntakeIntaked();

  // Periodic updates for logging and other state handling
  public default void updateInputs(ClawIOInputs inputs) {}
  ;
  ;
}
