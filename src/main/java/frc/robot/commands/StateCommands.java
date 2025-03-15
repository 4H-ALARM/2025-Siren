package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.constants.RobotConstants;
import frc.lib.constants.RobotConstants.ElevatorConstants;
import frc.lib.enums.robotStates;
import frc.robot.ToggleHandler;
import frc.robot.subsystems.bargemech.bargeMech;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;
import org.littletonrobotics.junction.Logger;

public class StateCommands {
  public static Command intake(
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler) {
    return new FunctionalCommand(
        // initialize()
        () -> {
          stateHandler.setState(robotStates.INTAKE);
        },
        // execute()
        () -> {},
        // end(interrupted)
        (interrupted) -> {
          stateHandler.setState(robotStates.RESTING);
        },
        // isFinished() -> bool
        () -> endEffector.getfrontIntaked(),
        // Subsystems
        elevator,
        endEffector,
        groundIntake);
  }

  public static Command intakeAlgae(GroundIntake groundIntake, StateHandler stateHandler) {
    return new FunctionalCommand(
        // initialize()
        () -> {
          stateHandler.setState(robotStates.GROUNDINTAKE);
        },
        // execute()
        () -> {},
        // end(interrupted)
        (interrupted) -> {
          stateHandler.setState(robotStates.GROUNDHOLD);
        },
        // isFinished() -> bool
        () -> false,
        // Subsystems
        groundIntake);
  }

  public static Command intakeCenterBackward(
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler) {

    return new FunctionalCommand(
        // initialize()
        () -> {
          stateHandler.setState(robotStates.INTAKECENTERBACKWARD);
        },
        // execute()
        () -> {},
        // end(interrupted)
        (interrupted) -> {
          stateHandler.setState(robotStates.RESTING);
        },
        // isFinished() -> bool
        () -> endEffector.getfrontIntaked() && endEffector.getbackIntaked(),
        // Subsystems
        elevator, endEffector, groundIntake);
  }

  public static Command intakeCenterForward(
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler) {

    return new FunctionalCommand(
        // initialize()
        () -> {
          stateHandler.setState(robotStates.INTAKECENTERFORWARD);
        },
        // execute()
        () -> {},
        // end(interrupted)
        (interrupted) -> {
          stateHandler.setState(robotStates.RESTING);
        },
        // isFinished() -> bool
        () -> {
          return !endEffector.getbackIntaked();
        },
        // Subsystems
        elevator, endEffector, groundIntake);
  }

  public static Command throwAlgae(GroundIntake groundIntake, StateHandler stateHandler) {
    return new FunctionalCommand(
        // initialize()
        () -> {
          stateHandler.setState(robotStates.GROUNDTHROW);
        },
        // execute()
        () -> {},
        // end(interrupted)
        (interrupted) -> {
          stateHandler.setState(robotStates.RESTING);
        },
        // isFinished() -> bool
        () -> {
          return false;
        },
        // Subsystems
        groundIntake);
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

    return new FunctionalCommand(
        // initialize()
        () -> {
          switch (stateHandler.getChosenlevel()) {
            case L1 -> stateHandler.setState(robotStates.L1SCORE);
            case L2 -> stateHandler.setState(robotStates.L2SCORE);
            case L3 -> stateHandler.setState(robotStates.L3SCORE);
            case L4 -> stateHandler.setState(robotStates.L4SCORE);
          }
        },
        // execute()
        () -> {},
        // end(interrupted)
        (interrupted) -> {
          if (disable.get()) {
            elevator.stopElevator();
            return;
          }
          stateHandler.setState(robotStates.RESTING);
        },
        // isFinished() -> bool
        () -> {
          return disable.get();
        },
        // Subsystems
        elevator, endEffector);
  }

  public static Command elevatorToChosenHeight(
      Elevator elevator,
      EndEffector endEffector,
      StateHandler stateHandler,
      ToggleHandler disable) {

    return new FunctionalCommand(
        // initialize()
        () -> {
          switch (stateHandler.getChosenlevel()) {
            case L1 -> stateHandler.setState(robotStates.L1SCORE);
            case L2 -> stateHandler.setState(robotStates.L2SCORE);
            case L3 -> stateHandler.setState(robotStates.L3SCORE);
            case L4 -> stateHandler.setState(robotStates.L4SCORE);
          }
        },
        // execute()
        () -> {
          Logger.recordOutput("Elevator/state", stateHandler.getChosenlevel());

          switch (stateHandler.getChosenlevel()) {
            case L1:
              stateHandler.setState(robotStates.L1PREPARE);
              elevator.setTargetPosition(RobotConstants.ElevatorConstants.CORAL_L1);
              break;
            case L2:
              stateHandler.setState(robotStates.L2PREPARE);
              elevator.setTargetPosition(RobotConstants.ElevatorConstants.CORAL_L2);
              break;
            case L3:
              stateHandler.setState(robotStates.L3PREPARE);
              elevator.setTargetPosition(RobotConstants.ElevatorConstants.CORAL_L3);
              break;
            case L4:
              stateHandler.setState(robotStates.L4PREPARE);
              elevator.setTargetPosition(RobotConstants.ElevatorConstants.CORAL_L4);
              break;
            case INTAKE:
              elevator.setTargetPosition(RobotConstants.ElevatorConstants.BOTTOM);
              break;
            default:
              break;
          }
        },
        // end(interrupted)
        (interrupted) -> {
          if (disable.get()) {
            elevator.stopElevator();
            return;
          }
          if (interrupted) {
            stateHandler.setState(robotStates.RESTING);
            elevator.setTargetPosition(RobotConstants.ElevatorConstants.BOTTOM);
          }
        },
        // isFinished() -> bool
        () -> {
          return elevator.isCloseEnough() || disable.get();
        },
        // Subsystems
        elevator, endEffector);
  }
}
