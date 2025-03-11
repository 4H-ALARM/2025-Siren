package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorIOSim implements ElevatorIO {

  public ElevatorIOSim() {}

  @Override
  public void move(double input) {}

  @Override
  public void stopElevator() {
    // TODO Auto-generated method stub
  }

  @Override
  public RelativeEncoder getEncoder() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void resetEncoder() {
    // TODO Auto-generated method stub

  }

  @Override
  public void moveToPoint(Rotation2d targetRot) {
    // TODO Auto-generated method stub

  }

  @Override
  public double getPercentRaised() {
    // TODO Auto-generated method stub
    return 0.0;
  }
}
