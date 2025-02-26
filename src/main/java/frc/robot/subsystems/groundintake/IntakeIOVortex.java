// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.constants.RobotConstants.GroundIntakeConstants;
import frc.robot.subsystems.groundintake.IntakeIO.IntakeIOInputs;

/** Add your docs here. */
public class IntakeIOVortex implements IntakeIO {

  private final SparkFlex tiltMotor;
  private final SparkFlex spinMotor;
  private final AbsoluteEncoder encoder;

  private final SparkClosedLoopController tiltController;

  // Connection debouncers
  private final Debouncer tiltConnectedDebounce = new Debouncer(0.5);
  private final Debouncer spinConnectedDebounce = new Debouncer(0.5);

  public IntakeIOVortex() {
    tiltMotor =
        new SparkFlex(
            GroundIntakeConstants.groundIntakeTiltMotorID,
            MotorType.fromId(GroundIntakeConstants.groundIntakeTiltMotorID));
    spinMotor =
        new SparkFlex(
            GroundIntakeConstants.groundIntakeSpinMotorID,
            MotorType.fromId(GroundIntakeConstants.groundIntakeSpinMotorID));

    SparkFlexConfig config = new SparkFlexConfig();

    config
        .closedLoop
        .pid(0, 0, 0)
        .maxOutput(0)
        .minOutput(0)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    encoder = tiltMotor.getAbsoluteEncoder();
    tiltController = tiltMotor.getClosedLoopController();
  }

  @Override
  public Rotation2d getGIAngle() {
    return Rotation2d.fromRotations(encoder.getPosition());
  }

  @Override
  public void setAngle(Rotation2d angle) {
    tiltController.setReference(angle.getRotations(), ControlType.kPosition);
  }

  @Override
  public void setSpeed(double speed) {
    spinMotor.set(speed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}
}
