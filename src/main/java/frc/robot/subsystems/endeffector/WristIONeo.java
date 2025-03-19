package frc.robot.subsystems.endeffector;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.constants.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class WristIONeo implements WristIO {
  private final SparkMax motor;
  private final SparkClosedLoopController pidController;
  private final AbsoluteEncoder encoder;

  // Constructor
  public WristIONeo() {

    motor = new SparkMax(RobotConstants.EndEffectorConstants.wristmotorID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    pidController = motor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config.absoluteEncoder.zeroOffset(0.2926049 + 0.3);
    config
        .closedLoop
        .pid(4, 0, 0.05)
        .minOutput(-1)
        .maxOutput(1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .maxMotion
        .allowedClosedLoopError(0.05)
        .maxAcceleration(30000)
        .maxVelocity(5600);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setAngle(Rotation2d angle) {
    pidController.setReference(angle.getRotations(), ControlType.kPosition);
  }

  @Override
  public double getRotation() {

    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void stopMotor() {
    motor.stopMotor();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.angle = getRotation();
    Logger.recordOutput("Wrist/angle", encoder.getPosition());
  }
}
