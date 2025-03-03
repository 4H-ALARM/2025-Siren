package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
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
import frc.lib.constants.RobotConstants.ElevatorConstants.elevatorState;
import org.littletonrobotics.junction.Logger;

public class ElevatorIONeo implements ElevatorIO {

  private final SparkMax leadMotor;
  private final SparkMax motor2;

  private final RelativeEncoder encoder;

  private final SparkClosedLoopController leadpid;
  private final SparkMaxConfig leadConfig;
  private final SparkMaxConfig motor2config;

  // private final DigitalInput limitswitchBottom;

  // private final DigitalInput bottomLimitSwitch;
  // private final DigitalInput topLimitSwitch;

  private final double rotationstoInches = 0.0;

  private elevatorState state = elevatorState.DEFAULT;

  public ElevatorIONeo() {

    // bottomLimitSwitch = new DigitalInput(RobotConstants.ElevatorConstants.bottomlimitswitchID);
    // topLimitSwitch = new DigitalInput(RobotConstants.ElevatorConstants.toplimitswitchID);

    leadMotor = new SparkMax(RobotConstants.ElevatorConstants.leadMotorID, MotorType.kBrushless);
    motor2 = new SparkMax(RobotConstants.ElevatorConstants.followerMotorID, MotorType.kBrushless);

    encoder = leadMotor.getEncoder();
    encoder.setPosition(0);
    leadpid = leadMotor.getClosedLoopController();

    leadConfig = new SparkMaxConfig();
    leadConfig
        .closedLoop
        .pid(0.075, 0, 0)
        .minOutput(-0.5)
        .maxOutput(0.5)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor2config = new SparkMaxConfig();
    motor2config.apply(leadConfig);
    motor2config.follow(leadMotor, true);
    motor2.configure(motor2config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void moveToState(RobotConstants.ElevatorConstants.elevatorState state) {
    boolean atLowestPoint = false;
    boolean atHighestPoint = false;
    // if (bottomLimitSwitch.get()) {
    //   atLowestPoint = true;
    // }
    // if (topLimitSwitch.get()) {
    //   atHighestPoint = true;
    // }
    this.state = state;
    moveToPoint(state.getTargetRotation2d());
  }

  @Override
  public void moveToPoint(Rotation2d targetRot) {
    // if (bottomLimitSwitch.get()
    //    && targetRot.getDegrees() > Rotation2d.fromRotations(encoder.getPosition()).getDegrees())
    // {
    //  stopElevator();
    // }
    // if (topLimitSwitch.get()
    //    && targetRot.getDegrees() < Rotation2d.fromRotations(encoder.getPosition()).getDegrees())
    // {
    //  stopElevator();
    // }
    leadpid.setReference(targetRot.getRotations(), ControlType.kPosition);
    // pid2.setReference((-getEncoder()), ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void move(double input) {
    double realinput = input * 0.2;

    if (realinput > 0.15) {
      realinput = 0.15;
    }

    if (realinput < -0.15) {
      realinput = -0.15;
    }

    // if (bottomLimitSwitch.get() && realinput < 0) {
    //   stopElevator();
    // }
    // if (topLimitSwitch.get() && realinput > 0) {
    //   stopElevator();
    // }

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
  public double getPercentRaised() {

    return (getEncoder().getPosition() / elevatorState.L4.getTargetRotation2d().getRotations());
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    Logger.recordOutput("elevator/encoder", getEncoder().getPosition());
    inputs.enumState = state.name();
  }
}
