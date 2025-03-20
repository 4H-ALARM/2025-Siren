package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ToggleHandler;
import frc.robot.commands.StateCommands.*;
import frc.robot.subsystems.bargemech.bargeMech;
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
      bargeMech bargeMech,
      StateHandler stateHandler,
      ToggleHandler elevatorDisable,
      ToggleHandler alignDisable) {
    super(
        Commands.parallel(
            // new ToClosestTargetPoseCommand(
            //     drive, alignDisable,
            // frc.lib.constants.RobotConstants.GeneralConstants.algaePoses),
            new ElevatorToChosenHeight(elevator, endEffector, stateHandler, elevatorDisable)),
        new DeAlgifyAtChosenHeight(
            elevator, endEffector, stateHandler, elevatorDisable, bargeMech));
  }
}
