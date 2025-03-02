// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.constants.RobotConstants.GroundIntakeConstants;
import frc.robot.subsystems.groundintake.IntakeIO.IntakeIOInputs;

/** Add your docs here. */
public class IntakeIOVortex implements IntakeIO {

  private final TalonFX tiltMotor;
  private final SparkFlex spinMotor;

  private PositionVoltage positionVoltage = new PositionVoltage(0.0);

  public IntakeIOVortex() {
    tiltMotor = new TalonFX(GroundIntakeConstants.groundIntakeTiltMotorID);
    spinMotor =
        new SparkFlex(
            GroundIntakeConstants.groundIntakeSpinMotorID,
            MotorType.fromId(GroundIntakeConstants.groundIntakeSpinMotorID));

    SparkFlexConfig config = new SparkFlexConfig();

    config.closedLoop.pid(0, 0, 0).maxOutput(0).minOutput(0);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    tiltMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public Rotation2d getGIAngle() {
    return Rotation2d.fromRotations(tiltMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void setAngle(Rotation2d angle) {
    positionVoltage.Position = angle.getRotations();
    tiltMotor.setControl(positionVoltage.withEnableFOC(true));
  }

  @Override
  public void setSpeed(double speed) {
    spinMotor.set(speed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}
}
