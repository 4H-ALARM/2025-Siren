package frc.robot.commands;

import static frc.lib.constants.RobotConstants.GeneralConstants.intakePoses;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.GeometryUtil;
import frc.robot.subsystems.drive.Drive;

public class ToIntakePoseCommand extends Command {
  private final Drive drive;
  private Command driveToPose;

  public ToIntakePoseCommand(Drive drive) {
    this.drive = drive;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive);

    driveToPose = new InstantCommand();
  }

  @Override
  public void initialize() {
    Pose2d closestpose = new Pose2d();
    double closestDistance = 900000000;
    for (int i = 0; i < intakePoses.length; i++) {
      double distance =
          GeometryUtil.toTransform2d(drive.getPose())
              .getTranslation()
              .getDistance(GeometryUtil.toTransform2d(intakePoses[i]).getTranslation());
      if (distance < closestDistance) {
        closestpose = intakePoses[i];
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
    return driveToPose.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    driveToPose.end(interrupted);
  }
}
