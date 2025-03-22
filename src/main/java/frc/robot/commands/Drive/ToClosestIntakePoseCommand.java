package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.constants.RobotConstants;
import frc.robot.ToggleHandler;
import frc.robot.subsystems.drive.Drive;

public class ToClosestIntakePoseCommand extends Command {
  private final Drive drive;
  private Command driveToPose;
  private boolean isNotBlue;
  private ToggleHandler disable;

  public ToClosestIntakePoseCommand(Drive drive, ToggleHandler disable) {
    this.drive = drive;
    this.disable = disable;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive);
    driveToPose = new InstantCommand();
  }

  @Override
  public void initialize() {
    Pose2d currentpose = drive.getPose();
    Pose2d targetpose =
        RobotConstants.GeneralConstants.calculateIntakePoses(
            currentpose.getX(), currentpose.getY(), 0.861, 0.628, 1);

    driveToPose = new AlignToPoseCommand(this.drive, targetpose);
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
