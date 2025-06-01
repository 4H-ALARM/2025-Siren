package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.constants.RobotConstants;
import frc.lib.util.BasePosition;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOKraken implements ElevatorIO {

  private final TalonFX leftKraken;
  // private final TalonFX followerKraken;

  private PositionVoltage control;

  private final CANdi bottomLimitSwitch = new CANdi(RobotConstants.ElevatorConstants.candi);

  private boolean limitSwitchBroke;
  // private final double rotationstoInches = 0.0;

  public static final double encoderLowerLimit = 0.0;
  public static final double encoderUpperLimit = 23;

  public ElevatorIOKraken() {

    Slot0Configs pidConfigs = new Slot0Configs().withKP(0.5).withKI(0.1).withKD(0);

    leftKraken = new TalonFX(RobotConstants.ElevatorConstants.elevatorLeft);
    // // followerKraken = new TalonFX(RobotConstants.ElevatorConstants.elevatorRight);
    leftKraken.setPosition(0);
    leftKraken.getConfigurator().apply(pidConfigs);
    // // followerKraken.setControl(new Follower(RobotConstants.ElevatorConstants.elevatorLeft,
    // true));
    control = new PositionVoltage(0); // .withEnableFOC(true);

    // limitSwitchBroke = false;
  }

  @Override
  public void periodic() {
    var basePosition =
        BasePosition.fromRange(
                encoderLowerLimit, encoderUpperLimit, leftKraken.getPosition().getValueAsDouble())
            .getValue();
    Logger.recordOutput("elevator/basePosition", basePosition);
    if (bottomLimitSwitch.getS1Closed().getValue()) {
      Logger.recordOutput("candi", true);
    } else {
      Logger.recordOutput("candi", false);
    }
  }

  public void setTargetPosition(BasePosition position) {

    // if (bottomLimitSwitch.get() && topLimitSwitch.get()) {
    //   limitSwitchBroke = true;
    // }

    // if (limitSwitchBroke) {
    //   leadMotor.stopMotor();
    //   followMotor.stopMotor();
    //   return;
    // }

    // if (bottomLimitSwitch.get()) {
    //   encoder.setPosition(encoderLowerLimit);
    // }
    // if (topLimitSwitch.get()) {
    //   encoder.setPosition(encoderUpperLimit);
    // }
    double targetEncoderPosition = position.toRange(encoderLowerLimit, encoderUpperLimit);
    Logger.recordOutput("Elevator/targetrot", targetEncoderPosition);
    Logger.recordOutput("Elevator/encoder", leftKraken.getPosition().getValueAsDouble());
    leftKraken.setControl(control.withPosition(targetEncoderPosition).withSlot(0));
  }

  public BasePosition getBasePosition() {
    return BasePosition.fromRange(
        encoderLowerLimit, encoderUpperLimit, leftKraken.getPosition().getValueAsDouble());
  }

  @Override
  public void move(double input) {
    double realinput = input * 0.5;

    if (realinput > 0.80) {
      realinput = 0.80;
    } else if (realinput < -0.80) {
      realinput = -0.80;
    }

    // if (bottomLimitSwitch.getS1Closed().getValue() && realinput > 0) {
    //   stopElevator();
    // }
    // if (topLimitSwitch.get() && realinput < 0) {
    //   stopElevator();
    // }
    Logger.recordOutput("Elevator/encoder", leftKraken.getPosition().getValueAsDouble());
    leftKraken.set(realinput);
    // followerKraken.set(realinput);
  }

  @Override
  public double getEncoder() {
    return leftKraken.getPosition().getValueAsDouble();
  }

  @Override
  public void stopElevator() {
    leftKraken.stopMotor();
  }

  @Override
  public void resetEncoder() {
    leftKraken.setPosition(0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}
}
