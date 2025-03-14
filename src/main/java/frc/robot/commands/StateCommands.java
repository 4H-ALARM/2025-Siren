package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.constants.RobotConstants.ElevatorConstants;
import frc.lib.enums.robotStates;
import frc.robot.ToggleHandler;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

public class StateCommands {
  public static Command intake(
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler) {
    return Commands.sequence(
            // like initialize()
            Commands.runOnce(
                () -> stateHandler.setState(robotStates.INTAKE),
                elevator,
                endEffector,
                groundIntake),
            // like execute() doing noting
            Commands.idle(elevator, endEffector, groundIntake)
                .until(() -> endEffector.getfrontIntaked()))
        // like end()
        .finallyDo(() -> stateHandler.setState(robotStates.RESTING));
  }

  public static Command intakeAlgae(GroundIntake groundIntake, StateHandler stateHandler) {
    return Commands.sequence(
            // like initialize()
            Commands.runOnce(() -> stateHandler.setState(robotStates.GROUNDINTAKE), groundIntake),
            // like execute() doing noting
            Commands.idle(groundIntake))
        // like end()
        .finallyDo(() -> stateHandler.setState(robotStates.GROUNDHOLD));
  }

  public static Command intakeCenterBackward(
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler) {
    return Commands.sequence(
            // like initialize()
            Commands.runOnce(
                () -> stateHandler.setState(robotStates.INTAKECENTERBACKWARD),
                elevator,
                endEffector,
                groundIntake),
            // like execute() doing noting
            Commands.idle(elevator, endEffector, groundIntake)
                .until(() -> endEffector.getfrontIntaked() && endEffector.getbackIntaked()))
        // like end()
        .finallyDo(() -> stateHandler.setState(robotStates.RESTING));
  }

  public static Command intakeCenterForward(
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler) {
    return Commands.sequence(
            // like initialize()
            Commands.runOnce(
                () -> stateHandler.setState(robotStates.INTAKECENTERFORWARD),
                elevator,
                endEffector,
                groundIntake),
            // like execute() doing noting
            Commands.idle(elevator, endEffector, groundIntake)
                .until(() -> endEffector.getfrontIntaked() && endEffector.getbackIntaked()))
        // like end()
        .finallyDo(() -> stateHandler.setState(robotStates.RESTING));
  }

  public static Command throwAlgae(GroundIntake groundIntake, StateHandler stateHandler) {
    return Commands.sequence(
            // like initialize()
            Commands.runOnce(() -> stateHandler.setState(robotStates.GROUNDTHROW), groundIntake),
            // like execute() doing noting
            Commands.idle(groundIntake))
        // like end()
        .finallyDo(() -> stateHandler.setState(robotStates.RESTING));
  }

  public static Command restingState(
      Elevator elevator, EndEffector endEffector, StateHandler stateHandler) {
    return Commands.runOnce(
        () -> {
          stateHandler.setState(robotStates.RESTING);
          elevator.setTargetPosition(ElevatorConstants.BOTTOM);
        },
        elevator,
        endEffector);
  }

  public static Command placeAtChosenHeight(
      Elevator elevator,
      EndEffector endEffector,
      StateHandler stateHandler,
      ToggleHandler disable) {
    return Commands.sequence(
            // like initialize()
            Commands.runOnce(
                () -> {
                  switch (stateHandler.getChosenlevel()) {
                    case L1 -> stateHandler.setState(robotStates.L1SCORE);
                    case L2 -> stateHandler.setState(robotStates.L2SCORE);
                    case L3 -> stateHandler.setState(robotStates.L3SCORE);
                    case L4 -> stateHandler.setState(robotStates.L4SCORE);
                  }
                },
                elevator,
                endEffector),
            // like isFinished() returning disabled status
            Commands.idle(elevator, endEffector).until(() -> disable.get()))
        // like end()
        .finallyDo(() -> {
            if (disable.get()) {
                elevator.stopElevator();
                return;
            }
            stateHandler.setState(robotStates.RESTING);
        });
  }
}
