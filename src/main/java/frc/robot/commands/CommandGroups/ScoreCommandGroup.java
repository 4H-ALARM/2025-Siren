package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.StateCommands.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
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
        new ParallelCommandGroup(
            // new ToClosestReefPoseCommand(drive),
            new ElevatorToChosenHeight(elevator, endEffector, stateHandler))
        // new PlaceAtChosenHeight(elevator, endEffector, stateHandler).withTimeout(1.5),
        // new ParallelCommandGroup(
        //     new SequentialCommandGroup(
        //         new InstantCommand().withTimeout(0.05),
        //         new Restingstate(elevator, endEffector, stateHandler)))
        );
  }
}
