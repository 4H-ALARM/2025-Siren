package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.enums.robotStates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

public class Intake extends Command {
  private final Elevator elevator;
  private final EndEffector endEffector;
  private final GroundIntake groundIntake;
  private final StateHandler stateHandler;

  public Intake(
      Elevator elevator, EndEffector endEffector, GroundIntake groundIntake, StateHandler handler) {
    this.elevator = elevator;
    this.endEffector = endEffector;
    this.groundIntake = groundIntake;
    this.stateHandler = handler;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.elevator, this.endEffector, this.groundIntake);
  }

  @Override
  public void initialize() {
    this.stateHandler.setState(robotStates.INTAKE);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return this.endEffector.getfrontIntaked();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      this.stateHandler.setState(robotStates.RESTING);
    }
    this.stateHandler.setState(robotStates.RESTING);
  }
}
