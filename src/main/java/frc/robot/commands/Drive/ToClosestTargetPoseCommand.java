package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.GeometryUtil;
import frc.robot.ToggleHandler;
import frc.robot.subsystems.drive.Drive;

public class ToClosestTargetPoseCommand extends Command {
  private final Drive drive;
  private Command driveToPose;
  private boolean isNotBlue;
  private ToggleHandler disable;
  private Pose2d[] poses;

  public ToClosestTargetPoseCommand(Drive drive, ToggleHandler disable, Pose2d[] poses) {
    this.drive = drive;
    this.disable = disable;
    this.poses = poses;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive);
    driveToPose = new InstantCommand();
  }

  @Override
  public void initialize() {
    Pose2d closestpose = new Pose2d();
    double closestDistance = 900000000;
    for (int i = 0; i < poses.length; i++) {
      Pose2d checkingPose = AllianceFlipUtil.apply(poses[i]);
      double distance =
          GeometryUtil.toTransform2d(drive.getPose())
              .getTranslation()
              .getDistance(GeometryUtil.toTransform2d(checkingPose).getTranslation());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestpose = checkingPose; // intakePoses[i];
      }
    }

    driveToPose = new AlignToPoseCommand(this.drive, closestpose);
    driveToPose.initialize();
  }

  @Override
  public void execute() {
    driveToPose.execute();
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return driveToPose.isFinished() || disable.get();
  }

  @Override
  public void end(boolean interrupted) {
    driveToPose.end(interrupted);
  }
}
