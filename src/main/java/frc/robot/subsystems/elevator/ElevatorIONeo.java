package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.lib.constants.RobotConstants;
import frc.lib.util.BasePosition;
import frc.lib.util.ValidatingDigitalInput;

import org.littletonrobotics.junction.Logger;

public class ElevatorIONeo implements ElevatorIO {

  private final SparkMax leadMotor;
  private final SparkMax followMotor;

  private final RelativeEncoder encoder;

  private final SparkClosedLoopController controller;
  private final SparkMaxConfig leadConfig;
  private final SparkMaxConfig followConfig;

  private final ValidatingDigitalInput bottomLimitSwitch;
  private final ValidatingDigitalInput topLimitSwitch;

  private final double encoderLowerLimit = 0.0;
  private final double encoderUpperLimit = 280.0;
  // private final double rotationstoInches = 0.0;

  public ElevatorIONeo() {

    bottomLimitSwitch = new ValidatingDigitalInput(RobotConstants.ElevatorConstants.bottomlimitswitchID, false);
    topLimitSwitch = new ValidatingDigitalInput(RobotConstants.ElevatorConstants.toplimitswitchID, false);

    leadMotor = new SparkMax(RobotConstants.ElevatorConstants.leadMotorID, MotorType.kBrushless);
    followMotor =
        new SparkMax(RobotConstants.ElevatorConstants.followerMotorID, MotorType.kBrushless);

    encoder = leadMotor.getEncoder();
    encoder.setPosition(0);

    controller = leadMotor.getClosedLoopController();

    leadConfig = new SparkMaxConfig();
    leadConfig
        // .apply(new EncoderConfig().inverted(true))
        .apply(
        new ClosedLoopConfig()
            .pid(0.15, 0, 0)
            .minOutput(-1)
            .maxOutput(1)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder));

    leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followConfig = new SparkMaxConfig();
    followConfig.follow(leadMotor, true);
    followConfig.apply(leadConfig);
    followMotor.configure(
        followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    var encoderPosition = encoder.getPosition();
    var basePosition =
        BasePosition.fromRange(encoderLowerLimit, encoderUpperLimit, encoderPosition).getValue();
    Logger.recordOutput("elevator/basePosition", basePosition);
  }

  public void setTargetPosition(BasePosition position) {
    if (bottomLimitSwitch.get()) {
      encoder.setPosition(encoderLowerLimit);
    }
    if (topLimitSwitch.get()) {
      encoder.setPosition(encoderUpperLimit);
    }
    double targetEncoderPosition = position.toRange(encoderLowerLimit, encoderUpperLimit);
    controller.setReference(targetEncoderPosition, ControlType.kPosition);
  }

  public BasePosition getBasePosition() {
    return BasePosition.fromRange(encoderLowerLimit, encoderUpperLimit, encoder.getPosition());
  }

  @Override
  public void move(double input) {
    double realinput = input * 0.5;

    if (realinput > 0.15) {
      realinput = 0.15;
    }

    if (realinput < -0.15) {
      realinput = -0.15;
    }

    if (bottomLimitSwitch.get() && realinput < 0) {
      stopElevator();
    }
    if (topLimitSwitch.get() && realinput > 0) {
      stopElevator();
    }
    Logger.recordOutput("Elevator/encoder", encoder.getPosition());
    leadMotor.set(realinput);
  }

  @Override
  public RelativeEncoder getEncoder() {
    return encoder;
  }

  @Override
  public void stopElevator() {
    leadMotor.stopMotor();
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}
}
