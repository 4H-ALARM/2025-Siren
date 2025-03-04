// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroundIntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.groundintake.GroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundIntakeStatic extends Command {
  GroundIntake groundIntake;
  /** Creates a new GroundIntakeStatic. */
  public GroundIntakeStatic(GroundIntake gi) {
    this.groundIntake = gi;
    addRequirements(this.groundIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.groundIntake.setAngle(0.1);
    this.groundIntake.setSpeed(-0.05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    groundIntake.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
