package frc.robot.subsystems.claw;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotConstants;

public class ClawIOVortex implements ClawIO {

  private final SparkFlex motor;
  private final SparkClosedLoopController m_controller;

  // Constructor
  public ClawIOVortex() {
    motor = new SparkFlex(RobotConstants.EndEffectorConstants.clawmotorID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .closedLoop
        .pid(0.0025, 0, 2)
        .minOutput(-1)
        .maxOutput(1)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxMotion
        .allowedClosedLoopError(0.5);

    m_controller = motor.getClosedLoopController();
  }

  @Override
  public void setSpeed(double speed) {
    motor.set(speed);
  }

  @Override
  public double getCurrentSpeed() {
    return motor.getEncoder().getVelocity();
  }

  @Override
  public double getPosition() {
    return motor.getEncoder().getPosition();
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }
}
