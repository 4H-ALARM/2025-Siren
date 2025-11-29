package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    double angle = 0;
  }

  public void setAngle(Rotation2d angle);

  public void stopMotor();

  public double getRotation();

  public double getVelocity();

  // Updates any periodic logging or state
  public default void updateInputs(WristIOInputs inputs) {}
}
