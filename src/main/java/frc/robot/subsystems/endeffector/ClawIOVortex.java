package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.lib.constants.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class ClawIOVortex implements ClawIO {

  private final SparkFlex motor;
  private final SparkClosedLoopController m_controller;

  private final CANrange frontCaNrange;
  private final CANrange intakeCaNrange;

  // Constructor
  public ClawIOVortex() {
    motor = new SparkFlex(RobotConstants.EndEffectorConstants.clawmotorID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .closedLoop
        .pid(1, 0, 0)
        .minOutput(-1)
        .maxOutput(1)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxMotion
        .allowedClosedLoopError(0.1);

    m_controller = motor.getClosedLoopController();

    frontCaNrange = new CANrange(RobotConstants.EndEffectorConstants.frontcanrange);
    frontCaNrange.getConfigurator().apply(new ProximityParamsConfigs().withProximityThreshold(0.1));

    intakeCaNrange = new CANrange(RobotConstants.EndEffectorConstants.intakecanrange);
    intakeCaNrange
        .getConfigurator()
        .apply(new ProximityParamsConfigs().withProximityThreshold(0.1));
  }

  @Override
  public void setSpeed(double rpm) {
    motor.set(rpm);
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
  public boolean getFrontIntaked() {

    return frontCaNrange.getIsDetected().getValue();
  }

  @Override
  public boolean getIntakeIntaked() {

    return intakeCaNrange.getIsDetected().getValue();
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }

  @Override
  public void updateInputs(ClawIOInputs inputs) {
    Logger.recordOutput("claw/canrangefront", frontCaNrange.getDistance().getValueAsDouble());
    Logger.recordOutput("claw/canrangefrontbool", frontCaNrange.getIsDetected().getValue());
    Logger.recordOutput("claw/canrangeintake", intakeCaNrange.getDistance().getValueAsDouble());
    Logger.recordOutput("claw/canrangeintakebool", intakeCaNrange.getIsDetected().getValue());
  }
}
