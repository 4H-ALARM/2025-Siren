package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.enums.robotStates;
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
}
