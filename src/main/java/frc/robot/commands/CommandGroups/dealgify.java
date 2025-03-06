// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ElevatorCommands.elevatorsetHeightDealgify;
import frc.robot.commands.EndEffector.IntakeClaw;
import frc.robot.commands.EndEffector.L2Wrist;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.claw.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class dealgify extends ParallelCommandGroup {
  /** Creates a new dealgify. */
  public dealgify(Elevator elevator, EndEffector endEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new elevatorsetHeightDealgify(elevator),
        new L2Wrist(endEffector),
        new IntakeClaw(endEffector));
  }
}
