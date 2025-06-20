package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ToggleHandler;
import frc.robot.commands.Drive.ToClosestTargetPoseCommand;
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
      StateHandler stateHandler,
      ToggleHandler elevatorDisable,
      ToggleHandler alignDisable) {
    super(
        new ParallelCommandGroup(
            new ToClosestTargetPoseCommand(
                drive, alignDisable, frc.lib.constants.RobotConstants.GeneralConstants.reefPoses),
            new ElevatorToChosenHeight(elevator, endEffector, stateHandler, elevatorDisable)),
        new PlaceAtChosenHeight(elevator, endEffector, stateHandler, elevatorDisable)
            .withTimeout(1),
        // new ParallelCommandGroup(
        //     DriveCommands.driveBackwards(drive).withTimeout(0.15),
        new Restingstate(elevator, endEffector, stateHandler));
  }
}
