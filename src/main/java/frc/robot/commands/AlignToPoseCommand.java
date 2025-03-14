package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.constants.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AlignToPoseCommand extends Command {
  private final Drive drive;
  private Pose2d target;

  // private final LoggedTunableNumber translationp;
  // private final LoggedTunableNumber translationi;
  // private final LoggedTunableNumber translationd;

  // private final LoggedTunableNumber headingp;
  // private final LoggedTunableNumber headingi;
  // private final LoggedTunableNumber headingd;

  private final ProfiledPIDController alignXController;
  private final ProfiledPIDController alignYController;
  private final ProfiledPIDController alignHeadingController;

  public AlignToPoseCommand(Drive drive, Pose2d targetPose) {
    this.drive = drive;
    this.target = targetPose;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive);

    // translationp = new LoggedTunableNumber("translationp", RobotConstants.DriveConstants.alignP);
    // translationi = new LoggedTunableNumber("translationi", RobotConstants.DriveConstants.alignI);
    // translationd = new LoggedTunableNumber("translationd", RobotConstants.DriveConstants.alignD);

    // headingp = new LoggedTunableNumber("headingp", RobotConstants.DriveConstants.headingP);
    // headingi = new LoggedTunableNumber("headingi", RobotConstants.DriveConstants.headingI);
    // headingd = new LoggedTunableNumber("headingd", RobotConstants.DriveConstants.headingD);

    alignXController =
        new ProfiledPIDController(
            RobotConstants.DriveConstants.alignP,
            RobotConstants.DriveConstants.alignI,
            RobotConstants.DriveConstants.alignD,
            new TrapezoidProfile.Constraints(
                RobotConstants.DriveConstants.maxSpeed, RobotConstants.DriveConstants.maxAccel));
    alignXController.setTolerance(RobotConstants.DriveConstants.translationRange);

    alignYController =
        new ProfiledPIDController(
            RobotConstants.DriveConstants.alignP,
            RobotConstants.DriveConstants.alignI,
            RobotConstants.DriveConstants.alignD,
            new TrapezoidProfile.Constraints(
                RobotConstants.DriveConstants.maxSpeed, RobotConstants.DriveConstants.maxAccel));
    alignYController.setTolerance(RobotConstants.DriveConstants.translationRange);

    alignHeadingController =
        new ProfiledPIDController(
            RobotConstants.DriveConstants.headingP,
            RobotConstants.DriveConstants.headingI,
            RobotConstants.DriveConstants.headingD,
            new TrapezoidProfile.Constraints(
                RobotConstants.DriveConstants.maxHeadingSpeed,
                RobotConstants.DriveConstants.maxHeadingAccel));
    alignHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    alignHeadingController.setTolerance(RobotConstants.DriveConstants.headingRange);
    // if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
    //   target = AllianceFlipUtil.apply(target);
    // }
    Logger.recordOutput("commands/targetpose", this.target);
  }

  @Override
  public void initialize() {
    Logger.recordOutput("commands/targetpose", this.target);
    alignXController.reset(drive.getPose().getX());
    alignYController.reset(drive.getPose().getY());
    alignHeadingController.reset(drive.getPose().getRotation().getRadians());
    // setTranslationPIDConstants(
    //     translationp.getAsDouble(), translationi.getAsDouble(), translationd.getAsDouble());
    // setHeadingPIDConstants(headingp.getAsDouble(), headingi.getAsDouble(),
    // headingd.getAsDouble());
  }

  @Override
  public void execute() {
    Logger.recordOutput("commands/targetpose", this.target);
    ChassisSpeeds commandedSpeeds =
        DriveCommands.driveFieldOriented(
            this.drive,
            this.alignXController.calculate(this.drive.getPose().getX(), this.target.getX()),
            this.alignYController.calculate(this.drive.getPose().getY(), this.target.getY()),
            -this.alignHeadingController.calculate(
                this.drive.getPose().getRotation().getRadians(),
                this.target.getRotation().getRadians()));
    this.drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(commandedSpeeds, drive.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return (isAligned());
  }

  @Override
  public void end(boolean interrupted) {
    this.drive.stop();
    ;
  }

  public void setTranslationPIDConstants(double kp, double ki, double kd) {
    alignXController.setPID(kp, ki, kd);
    alignYController.setPID(kp, ki, kd);
  }

  public void setHeadingPIDConstants(double kp, double ki, double kd) {
    alignHeadingController.setPID(kp, ki, kd);
  }

  public boolean isAligned() {
    Logger.recordOutput("headingisFinished", this.alignHeadingController.atGoal());
    Logger.recordOutput("xisFinished", this.alignXController.atGoal());
    Logger.recordOutput("yisFinished", this.alignYController.atGoal());
    Logger.recordOutput("commands/targetdegrees", target.getRotation().getDegrees());
    Logger.recordOutput("commands/actualdegree", this.drive.getRotation().getDegrees());
    if (alignXController.atGoal() && alignYController.atGoal() && alignHeadingController.atGoal()
    // && alignHeadingController.atSetpoint()

    // GeometryUtil.toTransform2d(this.drive.getPose())
    //       .getTranslation()
    //       .getDistance(GeometryUtil.toTransform2d(target).getTranslation())
    //   < 0.02
    ) {
      return true;
    }
    return false;
  }
}
