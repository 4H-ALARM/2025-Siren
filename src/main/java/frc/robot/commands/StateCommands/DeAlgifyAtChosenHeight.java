package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.constants.RobotConstants;
import frc.lib.enums.robotStates;
import frc.robot.ToggleHandler;
import frc.robot.subsystems.bargemech.bargeMech;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

public class DeAlgifyAtChosenHeight extends Command {
  private final Elevator elevator;
  private final bargeMech bargeMech;
  private final EndEffector endEffector;
  private final StateHandler stateHandler;
  private final ToggleHandler disable;

  public DeAlgifyAtChosenHeight(
      Elevator elevator,
      EndEffector endEffector,
      StateHandler handler,
      ToggleHandler disable,
      bargeMech bargeMech) {
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.stateHandler = handler;
    this.disable = disable;
    this.bargeMech = bargeMech;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.elevator, this.endEffector, this.bargeMech);
  }

  @Override
  public void initialize() {
    switch (this.stateHandler.getChosenlevel()) {
      case DEALGIFYLOW -> this.stateHandler.setState(robotStates.DEALGIFYLOW);
      case DEALGIFYHIGH -> this.stateHandler.setState(robotStates.DEALGIFYHIGH);
    }
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return this.disable.get();
  }

  @Override
  public void end(boolean interrupted) {
    if (disable.get()) {
      this.elevator.stopElevator();
      this.bargeMech.stopBarge();
      return;
    }
    this.stateHandler.setState(robotStates.RESTINGPOSTDEALGIFY);
    this.elevator.setTargetPosition(RobotConstants.ElevatorConstants.BOTTOM);
  }
}
