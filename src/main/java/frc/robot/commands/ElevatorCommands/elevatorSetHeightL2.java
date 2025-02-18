// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorSetHeightL2 extends Command {
  Elevator elevator;
  /** Creates a new elevatorSetHeightDefault. */
  public elevatorSetHeightL2(Elevator elevatorimpl) {
    this.elevator = elevatorimpl;
    addRequirements(this.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.elevator.setGoalState(RobotConstants.ElevatorConstants.elevatorState.L2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (this.elevator.getGoalState() == RobotConstants.ElevatorConstants.elevatorState.L2) {
    //   return true;
    // }
    return false;
  }
}
