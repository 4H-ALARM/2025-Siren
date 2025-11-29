package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.enums.LevelEnum;
import frc.robot.ToggleHandler;
import frc.robot.commands.StateCommands.ElevatorToChosenHeight;
import frc.robot.commands.StateCommands.Intake;
import frc.robot.commands.StateCommands.IntakeCenterBackward;
import frc.robot.commands.StateCommands.IntakeCenterForward;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

public class IntakeCommandGroup extends SequentialCommandGroup {
  public IntakeCommandGroup(
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler,
      ToggleHandler elevatordisable) {
    super(
        // new ParallelDeadlineGroup(
        //     new Intake(elevator, endEffector, groundIntake, stateHandler),
        //     new ToIntakePoseCommand(drive)
        //     ),
        Commands.runOnce(() -> stateHandler.setLevelEnum(LevelEnum.INTAKE)),
        new ElevatorToChosenHeight(elevator, endEffector, stateHandler, elevatordisable),
        new Intake(elevator, endEffector, groundIntake, stateHandler),
        new IntakeCenterForward(elevator, endEffector, groundIntake, stateHandler),
        new IntakeCenterBackward(elevator, endEffector, groundIntake, stateHandler));
  }
}
