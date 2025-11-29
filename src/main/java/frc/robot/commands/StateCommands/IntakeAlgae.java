// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.StateCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.enums.robotStates;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae extends Command {
  GroundIntake groundIntake;
  StateHandler stateHandler;
  /** Creates a new IntakeBarge. */
  public IntakeAlgae(GroundIntake groundIntake, StateHandler stateHandler) {
    this.groundIntake = groundIntake;
    this.stateHandler = stateHandler;
    addRequirements(this.groundIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.setState(robotStates.GROUNDINTAKE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.setState(robotStates.GROUNDHOLD);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
