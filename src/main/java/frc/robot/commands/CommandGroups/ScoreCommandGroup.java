package frc.robot.commands.CommandGroups;

import java.security.GeneralSecurityException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.ToClosestReefPoseCommand;
import frc.lib.constants.RobotConstants;
import frc.lib.constants.RobotConstants.DriveConstants;
import frc.robot.commands.DriveCommands;
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
        new ParallelCommandGroup(
            new ToClosestReefPoseCommand(drive, RobotConstants.GeneralConstants.reefPoses),
            new ElevatorToChosenHeight(elevator, endEffector, stateHandler)),
        new PlaceAtChosenHeight(elevator, endEffector, stateHandler).withTimeout(1),
        new ParallelCommandGroup(
            DriveCommands.driveBackwards(drive).withTimeout(0.8),
            new Restingstate(elevator, endEffector, stateHandler)));
  }

  public static Command scoreLeft(
    Drive drive,
    Elevator elevator,
    EndEffector endEffector,
    GroundIntake groundIntake,
    StateHandler stateHandler
  ) {
    return score(drive, elevator, endEffector, groundIntake, stateHandler, RobotConstants.GeneralConstants.reefPosesLeft);
  }

  public static Command scoreRight(
    Drive drive,
    Elevator elevator,
    EndEffector endEffector,
    GroundIntake groundIntake,
    StateHandler stateHandler
  ) {
    return score(drive, elevator, endEffector, groundIntake, stateHandler, RobotConstants.GeneralConstants.reefPosesRight);
  }

  public static Command scoreAll(
    Drive drive,
    Elevator elevator,
    EndEffector endEffector,
    GroundIntake groundIntake,
    StateHandler stateHandler
  ) {
    return score(drive, elevator, endEffector, groundIntake, stateHandler, RobotConstants.GeneralConstants.reefPoses);
  }

  public static Command score(
    Drive drive,
    Elevator elevator,
    EndEffector endEffector,
    GroundIntake groundIntake,
    StateHandler stateHandler,
    Pose2d[] poses
  ) {
    return Commands.sequence(
      Commands.parallel(
        new ToClosestReefPoseCommand(drive, RobotConstants.GeneralConstants.reefPoses),
        new ElevatorToChosenHeight(elevator, endEffector, stateHandler)
      ),
      new PlaceAtChosenHeight(elevator, endEffector, stateHandler).withTimeout(1),
      Commands.parallel(
        DriveCommands.driveBackwards(drive).withTimeout(0.8),
        new Restingstate(elevator, endEffector, stateHandler)
      )
    );
  }
}
