package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.constants.RobotConstants;
import frc.robot.ToggleHandler;
import frc.robot.commands.Drive.ToClosestPoseCommand;
import frc.robot.commands.StateCommands.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.groundintake.GroundIntake;
import frc.robot.subsystems.virtualsubsystems.statehandler.StateHandler;


public class AutoIntakeCommandGroup extends SequentialCommandGroup {
  public AutoIntakeCommandGroup(
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      GroundIntake groundIntake,
      StateHandler stateHandler,
      ToggleHandler elevatorDisable,
      ToggleHandler alignDisable) {
    super(
        new ParallelCommandGroup(
            new ToClosestPoseCommand(drive, alignDisable, RobotConstants.GeneralConstants.intakePoses), //intake poses is blank
            new ElevatorToChosenHeight(elevator, endEffector, stateHandler, elevatorDisable)), // to-do check elevator settings
        new IntakeCommandGroup(drive, elevator, endEffector, groundIntake, stateHandler, elevatorDisable)
    );
  }
}
