// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.virtualsubsystems.statehandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.config.robotstateconfig;
import frc.lib.enums.LevelEnum;
import frc.lib.enums.TargetPose;
import frc.lib.enums.robotStates;
import frc.lib.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class StateHandler extends VirtualSubsystem {
  /** Creates a new StateHandler. */
  private robotstateconfig state;

  private LevelEnum chosenlevel;

  public StateHandler() {
    this.state = new robotstateconfig();
    chosenlevel = LevelEnum.L4;
  }

  public robotstateconfig getState() {
    return this.state;
  }

  public LevelEnum getChosenlevel() {
    return this.chosenlevel;
  }

  public void setLevelEnum(LevelEnum level) {
    this.chosenlevel = level;
  }

  public void setElevatorHeight(Rotation2d elevatorheight) {
    this.setElevatorHeight(elevatorheight);
  }

  public void setWristRotation(Rotation2d wristRotation) {
    this.state.setWristRotation(wristRotation);
  }

  public void setGroundIntakeRot(Rotation2d groundIntakeRot) {
    this.state.setGroundIntakeRot(groundIntakeRot);
  }

  public void setGroundIntakeSpeed(double groundIntakeSpeed) {
    this.state.setGroundIntakeSpeed(groundIntakeSpeed);
  }

  public void setClawSpeed(double clawSpeed) {
    this.state.setClawSpeed(clawSpeed);
  }

  public void setTargetPose(TargetPose targetPose) {
    this.state.setTargetPose(targetPose);
  }

  public void setState(robotStates input) {
    this.state = input.getRobotStateConfig();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      this.state.setDisabled(true);
    }
    if (this.state.isDisabled()) {
      this.state.setDisabled(false);
    }

    Logger.recordOutput("state", this.state.getElevatorHeight());
  }
}
