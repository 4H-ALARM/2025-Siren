// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.constants.RobotConstants;
import frc.lib.enums.robotStates;
import frc.robot.subsystems.bargemech.bargeMech;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DealgifyState extends Command {
  private StateHandler stateHandler;
  private bargeMech barge;
  private Elevator elevator;
  private EndEffector endEffector;
  /** Creates a new DealgifyState. */
  public DealgifyState(
      StateHandler stateHandler, bargeMech barge, Elevator elevator, EndEffector endEffector) {
    this.stateHandler = stateHandler;
    this.barge = barge;
    this.elevator = elevator;
    this.endEffector = endEffector;
    addRequirements(barge, elevator, endEffector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (this.stateHandler.getChosenlevel()) {
      case DEALGIFYLOW:
        this.stateHandler.setState(robotStates.DEALGIFYLOW);
        this.elevator.setTargetPosition(RobotConstants.ElevatorConstants.DEALGIFYLOW);
        break;
      case DEALGIFYHIGH:
        this.stateHandler.setState(robotStates.DEALGIFYHIGH);
        this.elevator.setTargetPosition(RobotConstants.ElevatorConstants.DEALGIFYHIGH);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.stateHandler.setState(robotStates.RESTING);
    this.elevator.setTargetPosition(RobotConstants.ElevatorConstants.BOTTOM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
