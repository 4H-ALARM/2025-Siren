// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANdi;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.RobotConstants;
import frc.lib.constants.RobotConstants.ElevatorConstants;
import frc.lib.util.BasePosition;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  /** Creates a new Elevator. */
  private final ElevatorIO elevator;

  private ElevatorIOInputsAutoLogged elevatorinputs;
  private final StateHandler stateHandler;

  private TrapezoidProfile profile;
  private ElevatorFeedforward feedforward;
  private Timer profileTimer;

  private final CANdi candi = new CANdi(RobotConstants.ElevatorConstants.candi);

  private double targetRotation;

  private BasePosition ElevatorPositionNormalized;
  private State t0State;

  public Elevator(ElevatorIO elevator, StateHandler handler) {
    this.elevator = elevator;
    elevatorinputs = new ElevatorIOInputsAutoLogged();
    this.stateHandler = handler;

    profile = new TrapezoidProfile(new Constraints(200, 300));
    feedforward = new ElevatorFeedforward(0, 0.0, 0);
    profileTimer = new Timer();
    profileTimer.start();

    ElevatorPositionNormalized = new BasePosition(0.0);
    t0State =
        new State(
            this.elevator.getEncoder().getPosition(), this.elevator.getEncoder().getVelocity());
  }

  public void setTargetPosition(BasePosition position) {
    if (ElevatorPositionNormalized.toRange(
            RobotConstants.ElevatorConstants.encoderLowerLimit,
            RobotConstants.ElevatorConstants.encoderUpperLimit)
        != position.toRange(
            RobotConstants.ElevatorConstants.encoderLowerLimit,
            RobotConstants.ElevatorConstants.encoderUpperLimit)) {
      profileTimer.reset();
      t0State =
          new State(
              this.elevator.getEncoder().getPosition(), this.elevator.getEncoder().getVelocity());
    }
    ElevatorPositionNormalized = position;

    profileTimer.start();
  }

  public BasePosition getBasePosition() {
    return elevator.getBasePosition();
  }

  public void moveElevator(double input) {
    this.elevator.move(input);
  }

  public void stopElevator() {
    elevator.stopElevator();
    profileTimer.stop();
  }

  public void resetEncoder() {
    this.elevator.resetEncoder();
  }

  public boolean isCloseEnough() {

    double encoderposition = elevator.getEncoder().getPosition();

    if (Math.abs(
            encoderposition
                - ElevatorPositionNormalized.toRange(
                    RobotConstants.ElevatorConstants.encoderLowerLimit,
                    RobotConstants.ElevatorConstants.encoderUpperLimit))
        < ElevatorConstants.closeEnoughRange) {
      Logger.recordOutput("Elevator/closeenough", true);
      Logger.recordOutput(
          "difference",
          encoderposition
              - ElevatorPositionNormalized.toRange(
                  RobotConstants.ElevatorConstants.encoderLowerLimit,
                  RobotConstants.ElevatorConstants.encoderUpperLimit));
      return true;
    }
    Logger.recordOutput("Elevator/closeenough", false);
    return false;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Elevator/state", this.stateHandler.getChosenlevel());
    elevator.periodic();

    if (stateHandler.getState().isDisabled()) {
      profileTimer.stop();
      elevator.stopElevator();
    } // else if (candi.getS1Closed().getValue()) {
    // profileTimer.stop();
    // elevator.stopElevator();
    // Logger.recordOutput("candi", true);
    // }
    else {
      profileTimer.start();
      // Logger.recordOutput("candi", false);
    }
    // if (profileTimer.isRunning()) {

    Rotation2d targetRotation =
        Rotation2d.fromRotations(
            profile.calculate(
                    profileTimer.getTimestamp(),
                    t0State,
                    new State(
                        ElevatorPositionNormalized.toRange(
                            RobotConstants.ElevatorConstants.encoderLowerLimit,
                            RobotConstants.ElevatorConstants.encoderUpperLimit),
                        0))
                .position);

    elevator.setTargetPosition(
        BasePosition.fromRange(
            RobotConstants.ElevatorConstants.encoderLowerLimit,
            RobotConstants.ElevatorConstants.encoderUpperLimit,
            targetRotation.getRotations()));
    // }

    elevator.updateInputs(elevatorinputs);
    Logger.processInputs("Elevator", elevatorinputs);
  }
}
