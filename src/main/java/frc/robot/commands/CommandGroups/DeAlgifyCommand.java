package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ToggleHandler;
import frc.robot.commands.Drive.ToClosestReefPoseCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.StateCommands.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

public class DeAlgifyCommand extends SequentialCommandGroup {
  public DeAlgifyCommand(
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler,
      ToggleHandler elevatorDisable,
      ToggleHandler alignDisable) {
    super(
        new ParallelCommandGroup(
            new ToClosestReefPoseCommand(drive, alignDisable, frc.lib.constants.RobotConstants.GeneralConstants.algaePoses),
            new ElevatorToChosenHeight(elevator, endEffector, stateHandler, elevatorDisable)),
        new PlaceAtChosenHeight(elevator, endEffector, stateHandler, elevatorDisable)
            .withTimeout(1),
        new ParallelCommandGroup(
            DriveCommands.driveBackwards(drive).withTimeout(0.8),
            new Restingstate(elevator, endEffector, stateHandler)));
  }
}
