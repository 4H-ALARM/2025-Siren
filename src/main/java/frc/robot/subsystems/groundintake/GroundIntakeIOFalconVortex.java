// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.constants.RobotConstants;

/** Add your docs here. */
public class GroundIntakeIOFalconVortex implements GroundIntakeIO {

  private final TalonFX tiltMotor;
  private final SparkFlex spinMotor;
  private final PositionVoltage control;

  public GroundIntakeIOFalconVortex() {

    Slot0Configs pidConfigs =
        new Slot0Configs()
            .withKP(0.8)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKA(0.00)
            .withKS(0.00)
            .withKV(1.08)
            .withKG(0.17);

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(60);

    tiltMotor = new TalonFX(RobotConstants.GroundIntakeConstants.tiltMotorID, "Drive");
    tiltMotor.setPosition(0);
    tiltMotor.getConfigurator().apply(pidConfigs);
    spinMotor =
        new SparkFlex(RobotConstants.GroundIntakeConstants.spinMotorID, MotorType.kBrushless);
    control = new PositionVoltage(0).withEnableFOC(false);
  }

  @Override
  public void setAngle(Rotation2d target) {
    tiltMotor.setControl(control.withPosition(target.getRotations()).withSlot(0));
  }

  // @Override
  // public void moveAngle(double input) {
  //   tiltMotor.set(input);
  // }

  @Override
  public void setSpeed(double speed) {
    spinMotor.set(speed);
  }

  @Override
  public void stopMotors() {
    spinMotor.stopMotor();
    tiltMotor.stopMotor();
  }

  // @Override
  // public void resetEncoder() {
  //   tiltMotor.setPosition(0);
  // }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {
    org.littletonrobotics.junction.Logger.recordOutput(
        "groundintake/encoder", tiltMotor.getPosition().getValueAsDouble());
  }
}
