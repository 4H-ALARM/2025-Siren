package frc.robot.subsystems.bargemech;

import org.littletonrobotics.junction.AutoLog;

public interface bargeIO {
  @AutoLog
  public static class BargeIOInputs {}

  public void intake(double speed);

  public void outake(double speed);

  public void stopMotor();

  // Periodic updates for logging and other state handling
  public default void updateInputs(BargeIOInputs inputs) {}
  ;
  ;
}
