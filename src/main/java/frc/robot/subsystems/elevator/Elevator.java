// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANdi;
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

  private final CANdi candi = new CANdi(RobotConstants.ElevatorConstants.candi);

  private double targetRotation;

  private BasePosition ElevatorPositionNormalized;

  public Elevator(ElevatorIO elevator, StateHandler handler) {
    this.elevator = elevator;
    elevatorinputs = new ElevatorIOInputsAutoLogged();
    this.stateHandler = handler;

    ElevatorPositionNormalized = new BasePosition(0.0);
  }

  public void setTargetPosition(BasePosition position) {
    if (ElevatorPositionNormalized.toRange(
            RobotConstants.ElevatorConstants.encoderLowerLimit,
            RobotConstants.ElevatorConstants.encoderUpperLimit)
        != position.toRange(
            RobotConstants.ElevatorConstants.encoderLowerLimit,
            RobotConstants.ElevatorConstants.encoderUpperLimit)) {
      elevator.setTargetPosition(position);
    }
    ElevatorPositionNormalized = position;
  }

  public BasePosition getBasePosition() {
    return elevator.getBasePosition();
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

  public boolean isCloseEnough() {

    double encoderposition = elevator.getEncoder();

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

    // if (stateHandler.getState().isDisabled()) {
    //   elevator.stopElevator();
    // } // else if (candi.getS1Closed().getValue()) {

    elevator.setTargetPosition(ElevatorPositionNormalized);

    elevator.updateInputs(elevatorinputs);
    Logger.processInputs("Elevator", elevatorinputs);
  }
}
