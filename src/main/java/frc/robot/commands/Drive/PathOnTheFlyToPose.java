// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathOnTheFlyToPose extends Command {
  Drive drive;
  Pose2d target;
  PPHolonomicDriveController customPID;
  PIDController customPIDX;
  PIDController customPIDY;
  PIDController customPIDR;
  boolean usePathPlanner;
  PathConstraints constraints;
  List<Waypoint> waypoints;
  Command pathplannerCommand;
  /** Creates a new PathOnTheFlyToPose. */
  public PathOnTheFlyToPose(Drive drive, Pose2d targetPose) {
    this.drive = drive;
    this.target = targetPose;
    this.customPIDX = new PIDController(40, 0.1, 0.1);
    this.customPIDY = new PIDController(40, 0.1, 0.1);
    this.customPIDR = new PIDController(75, 0.1, 4);
    this.customPIDX.setTolerance(0.01);
    this.customPIDY.setTolerance(0.01);
    this.customPIDR.setTolerance(1);
    this.usePathPlanner = true;
    this.constraints = PathConstraints.unlimitedConstraints(12);
    this.waypoints =
        PathPlannerPath.waypointsFromPoses(new Pose2d(0.02, 0, new Rotation2d()), new Pose2d());
    this.pathplannerCommand = new InstantCommand();
    addRequirements(this.drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Math.abs(drive.getPose().getX() - this.target.getX()) > 0.5
        && Math.abs(drive.getPose().getY() - this.target.getY()) > 0.5) {
      usePathPlanner = true;
      this.pathplannerCommand =
          AutoBuilder.followPath(
              new PathPlannerPath(
                  this.waypoints,
                  constraints,
                  null,
                  new GoalEndState(0, this.target.getRotation())));
      this.pathplannerCommand.initialize();
      // this.waypoints =
      //     PathPlannerPath.waypointsFromPoses(
      //         this.drive.getPose().interpolate(this.target, 0.5), this.target);

    } else {
      usePathPlanner = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (usePathPlanner) {
      this.pathplannerCommand.execute();
    } else {
      this.drive.runVelocity(
          new ChassisSpeeds(
              this.customPIDX.calculate(this.drive.getPose().getX(), this.target.getX()),
              this.customPIDY.calculate(this.drive.getPose().getY(), this.target.getY()),
              this.customPIDR.calculate(
                  this.drive.getPose().getRotation().getRadians(),
                  this.target.getRotation().getRadians())));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.pathplannerCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
