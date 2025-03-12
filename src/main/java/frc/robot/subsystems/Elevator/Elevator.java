// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.RobotConstants.ElevatorConstants;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIO elevator;

  private ElevatorIOInputsAutoLogged elevatorinputs;
  private final StateHandler stateHandler;

  private ProfiledPIDController pidController;

  private double targetRotation;

  public Elevator(ElevatorIO elevator, StateHandler handler) {
    this.elevator = elevator;
    elevatorinputs = new ElevatorIOInputsAutoLogged();
    this.stateHandler = handler;

    targetRotation = this.stateHandler.getState().getElevatorHeight().getRotations();

    pidController =
        new ProfiledPIDController(0.15, 0, 0, new TrapezoidProfile.Constraints(2000, 20000));
  }

  public void moveElevator(double input) {
    this.elevator.move(input);
  }

  public void stopElevator() {
    elevator.stopElevator();
  }

  public void resetEncoder() {
    this.elevator.resetEncoder();
  }

  public double getPercentRaised() {
    return elevator.getPercentRaised();
  }

  public boolean isCloseEnough() {

    double encoderposition = elevator.getEncoder().getPosition();
    double targetposition = this.stateHandler.getState().getElevatorHeight().getRotations();

    if (targetposition < (encoderposition * (4 + ElevatorConstants.closeEnoughPercent))
        && targetposition > (encoderposition * (4 + ElevatorConstants.closeEnoughPercent))) {
      return true;
    }

    return false;
  }

  public void resetPID() {
    pidController.reset(elevator.getEncoder().getPosition());
  }

  @Override
  public void periodic() {

    if (stateHandler.getState().isDisabled()) {
      elevator.stopElevator();
    } else {
      elevator.move(
          pidController.calculate(
              elevator.getEncoder().getPosition(),
              this.stateHandler.getState().getElevatorHeight().getRotations()));
    }

    elevator.updateInputs(elevatorinputs);
    Logger.processInputs("Elevator", elevatorinputs);
    Logger.recordOutput("elevator/encoder", this.elevator.getEncoder().getPosition());
  }
}
