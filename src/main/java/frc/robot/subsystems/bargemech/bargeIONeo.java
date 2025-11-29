package frc.robot.subsystems.bargemech;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class bargeIONeo implements bargeIO {
  private final SparkMax motor1;
  private final SparkMax motor2;

  // Constructor
  public bargeIONeo() {

    motor1 = new SparkMax(10, MotorType.kBrushless);
    motor2 = new SparkMax(11, MotorType.kBrushless);
  }

  @Override
  public void intake(double speed) {
    motor1.set(speed);
    motor2.set(-speed);
  }

  @Override
  public void outake(double speed) {
    motor1.set(-speed);
    motor2.set(speed);
  }

  @Override
  public void stopMotor() {
    motor1.stopMotor();
    motor2.stopMotor();
  }
}
