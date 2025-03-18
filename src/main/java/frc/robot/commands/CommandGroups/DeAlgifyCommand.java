package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ToggleHandler;
import frc.robot.commands.Drive.ToClosestReefPoseCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.StateCommands.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.bargemech.bargeMech;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;

public class DeAlgifyCommand {
  public static Command deAlgifyCommand(
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      bargeMech bargeMech,
      StateHandler stateHandler,
      ToggleHandler elevatorDisable,
      ToggleHandler alignDisable) {
    return Commands.sequence(
        Commands.parallel(
            new ToClosestReefPoseCommand(
                drive, alignDisable, frc.lib.constants.RobotConstants.GeneralConstants.algaePoses),
            new ElevatorToChosenHeight(elevator, endEffector, stateHandler, elevatorDisable)),
        new DeAlgifyAtChosenHeight(elevator, endEffector, stateHandler, elevatorDisable, bargeMech)
            .withTimeout(1),
        Commands.parallel(
            DriveCommands.driveBackwards(drive).withTimeout(0.8),
            new Restingstate(elevator, endEffector, stateHandler)));
  }
}

