package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.GeometryUtil;
import frc.robot.ToggleHandler;
import frc.robot.subsystems.drive.Drive;

public class ToClosestPoseCommand extends Command {
  private final Drive drive;
  private Command alignToPose;
  private ToggleHandler disable;
  private Pose2d[] poses;

  /**
   * Given a list of potential poses, calculate the closest one then align to it.
   * 
   * @param drive The drive subsystem
   * @param disable Whether to disable the command
   * @param poses The list of poses to check
   */
  public ToClosestPoseCommand(Drive drive, ToggleHandler disable, Pose2d[] poses) {
    this.drive = drive;
    this.disable = disable;
    this.poses = poses;
    addRequirements(this.drive);
    alignToPose = new InstantCommand();
  }

  /**
   * Calculate the nearest target pose from the list of poses, then initialize the alignToPose command
   */
  @Override
  public void initialize() {
    Pose2d closestPose = new Pose2d();
    double closestDistance = 900000000;
    var drivePose = drive.getPose();

    for (int i = 0; i < poses.length; i++) {
      Pose2d checkingPose = AllianceFlipUtil.apply(poses[i]);
      double distance =
          GeometryUtil.toTransform2d(drivePose)
              .getTranslation()
              .getDistance(GeometryUtil.toTransform2d(checkingPose).getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = checkingPose;
      }
    }

    // After finding the closest pose, delegate to alignToPose
    alignToPose = DriveCommands.alignToPose(this.drive, closestPose);
    alignToPose.initialize();
  }

  @Override
  public void execute() {
    alignToPose.execute();
  }

  @Override
  public boolean isFinished() {
    return alignToPose.isFinished() || disable.get();
  }

  @Override
  public void end(boolean interrupted) {
    alignToPose.end(interrupted);
  }
}
