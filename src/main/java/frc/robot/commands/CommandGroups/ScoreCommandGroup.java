package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.ToClosestReefPoseCommand;
import frc.robot.commands.StateCommands.*;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

public class ScoreCommandGroup extends SequentialCommandGroup {
  public ScoreCommandGroup(
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler) {
    super(
        // new ElevatorToChosenHeight(elevator, endEffector, stateHandler)
        new ParallelDeadlineGroup(
            new ToClosestReefPoseCommand(drive),
            new ElevatorToChosenHeight(elevator, endEffector, stateHandler)),
        new PlaceAtChosenHeight(elevator, endEffector, stateHandler));
  }
}
