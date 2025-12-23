// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundintake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Simulated ground intake using AdvantageKit's LoggedMechanism2d. */
public class GroundIntakeIOSim implements GroundIntakeIO {

  private static final double ARM_LENGTH_METERS = 0.9;
  private static final double ROLLER_LENGTH_METERS = 0.25;
  private static final double MAX_DEG_PER_SEC = 360.0;

  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d armLigament;
  private final LoggedMechanismLigament2d rollerLigament;

  private Rotation2d targetRotation = new Rotation2d();
  private Rotation2d currentRotation = new Rotation2d();
  private double currentSpeed = 0.0;

  private double lastUpdateTimestamp = Timer.getFPGATimestamp();

  public GroundIntakeIOSim() {
    mechanism = new LoggedMechanism2d(2.0, 1.5);
    LoggedMechanismRoot2d root = mechanism.getRoot("GroundIntakeRoot", 0.75, 0.25);
    armLigament = root.append(new LoggedMechanismLigament2d("Arm", ARM_LENGTH_METERS, 0));
    rollerLigament =
        armLigament.append(new LoggedMechanismLigament2d("Roller", ROLLER_LENGTH_METERS, 0));

    armLigament.setLineWeight(4);
    rollerLigament.setLineWeight(6);
    rollerLigament.setColor(new Color8Bit(255, 140, 0));
  }

  @Override
  public void setAngle(Rotation2d targetRotation) {
    this.targetRotation = targetRotation;
  }

  @Override
  public void setSpeed(double speed) {
    currentSpeed = speed;
  }

  @Override
  public void stopMotors() {
    currentSpeed = 0.0;
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {
    stepSimulation();
    updateMechanism();

    inputs.tiltMotorConnected = true;
    inputs.spinMotorConnected = true;
    inputs.Rotation = currentRotation.getRotations();
    inputs.speed = currentSpeed;

    Logger.recordOutput("groundintake/sim/rotation", inputs.Rotation);
    Logger.recordOutput("groundintake/sim/speed", currentSpeed);
    Logger.recordOutput("Mechanisms/GroundIntake", mechanism);
    ArrayList<Pose3d> articulatedPoses = mechanism.generate3dMechanism();
    int n = 0;
    while (n < articulatedPoses.size()) {
      Logger.recordOutput("Mechanisms/GroundIntake" + n, articulatedPoses.get(n));
      n++;
    }
  }

  private void stepSimulation() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastUpdateTimestamp;
    lastUpdateTimestamp = now;

    double maxStepDegrees = MAX_DEG_PER_SEC * dt;
    double errorDegrees = targetRotation.minus(currentRotation).getDegrees();
    double clampedStep =
        Math.max(Math.min(errorDegrees, maxStepDegrees), -maxStepDegrees); // clamp slew rate
    currentRotation = currentRotation.plus(Rotation2d.fromDegrees(clampedStep));
  }

  private void updateMechanism() {
    armLigament.setAngle(currentRotation.getDegrees());
    rollerLigament.setAngle(currentRotation.getDegrees());
  }
}
