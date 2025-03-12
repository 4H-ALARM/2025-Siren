// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.RobotConstants;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

  private final ClawIO claw;
  private final WristIO wrist;

  private final StateHandler stateHandler;

  private final TrapezoidProfile profile;
  private final Timer timer;

  private WristIOInputsAutoLogged wristIOInputsAutoLogged;
  private ClawIOInputsAutoLogged clawIOInputsAutoLogged;

  private double targetAngle;

  /** Creates a new EndEffector. */
  public EndEffector(ClawIO clawimpl, WristIO wristimpl, StateHandler handler) {
    this.claw = clawimpl;
    this.wrist = wristimpl;
    this.stateHandler = handler;

    this.wristIOInputsAutoLogged = new WristIOInputsAutoLogged();
    this.clawIOInputsAutoLogged = new ClawIOInputsAutoLogged();

    profile = new TrapezoidProfile(new Constraints(10, 100));
    timer = new Timer();
    timer.start();

    targetAngle = this.stateHandler.getState().getWristRotation().getRotations();
  }

  public void setWristAngle(Rotation2d rot) {
    timer.reset();
  }

  public void setClawSpeed(double speed) {}

  public void stopWrist() {
    this.wrist.stopMotor();
  }

  public void stopClaw() {
    this.claw.stopMotor();
  }

  public boolean getfrontIntaked() {
    return claw.getFrontIntaked();
  }

  public boolean getbackIntaked() {
    return claw.getIntakeIntaked();
  }

  @Override
  public void periodic() {
    if (targetAngle != this.stateHandler.getState().getWristRotation().getRotations()) {
      timer.reset();
      targetAngle = this.stateHandler.getState().getWristRotation().getRotations();
    }

    wrist.setAngle(
        Rotation2d.fromRotations(
            profile.calculate(
                    timer.getTimestamp(),
                    new State(this.wrist.getRotation(), this.wrist.getVelocity()),
                    new State(this.stateHandler.getState().getWristRotation().getRotations(), 0))
                .position));

    // wrist.setAngle(this.stateHandler.getState().getWristRotation());

    claw.setSpeed(this.stateHandler.getState().getClawSpeed());

    this.claw.updateInputs(this.clawIOInputsAutoLogged);
    Logger.processInputs("Endeffector/claw", clawIOInputsAutoLogged);
    this.wrist.updateInputs(this.wristIOInputsAutoLogged);
    Logger.processInputs("Endeffector/wrist", wristIOInputsAutoLogged);
    Logger.recordOutput("CartesianPoses", RobotConstants.GeneralConstants.reefPoses);
  }
}
