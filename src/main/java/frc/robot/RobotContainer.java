// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.constants.SwerveConstants;
import frc.robot.commands.CommandGroups.IntakeFull;
import frc.robot.commands.CommandGroups.dealgify;
import frc.robot.commands.CommandGroups.scorel2;
import frc.robot.commands.CommandGroups.scorel3;
import frc.robot.commands.CommandGroups.scorel4;
import frc.robot.commands.Drive.DriveCommands;
import frc.robot.commands.Drive.PathOnTheFlyToPose;
import frc.robot.commands.EndEffector.OutakeClaw;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIONeo;
import frc.robot.subsystems.claw.ClawIOVortex;
import frc.robot.subsystems.claw.EndEffector;
import frc.robot.subsystems.claw.WristIONeo;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final Vision vision;

  private final Elevator elevator;

  private final EndEffector endEffector;

  private final Climber climber;

  List<Waypoint> waypoints =
      PathPlannerPath.waypointsFromPoses(
          new Pose2d(0.1, 0.000, Rotation2d.fromDegrees(0)),
          new Pose2d(0.05, 0.0000, Rotation2d.fromDegrees(0)));
  PathConstraints constraints = PathConstraints.unlimitedConstraints(12);

  PathPlannerPath path =
      new PathPlannerPath(
          waypoints,
          constraints,
          null, // The ideal starting state, this is only relevant for pre-planned paths, so can
          // be null for on-the-fly paths.
          new GoalEndState(
              0.0,
              Rotation2d.fromDegrees(
                  180)) // Goal end state. You can set a holonomic rotation here. If using a
          // differential drivetrain, the rotation will have no effect.
          );

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController copilot = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public final IntakeFull intakecommand;
  public final scorel2 l2command;
  public final scorel3 l3command;
  public final scorel4 l4command;
  public final dealgify dealgifycommand;
  public final OutakeClaw drop;
  public final PathOnTheFlyToPose toOrigin;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(SwerveConstants.FrontLeft),
                new ModuleIOTalonFX(SwerveConstants.FrontRight),
                new ModuleIOTalonFX(SwerveConstants.BackLeft),
                new ModuleIOTalonFX(SwerveConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement // ,
                // new VisionIOPhotonVision(camera1Name, robotToCamera1),
                // new VisionIOPhotonVision(camera2Name, robotToCamera2),
                // new VisionIOPhotonVision(camera3Name, robotToCamera3),
                // new VisionIOPhotonVision(camera4Name, robotToCamera4),
                // new VisionIOLimelight(limelightName, drive::getRotation)
                );

        elevator = new Elevator(new ElevatorIONeo());

        climber = new Climber(new ClimberIOKraken());

        endEffector = new EndEffector(new ClawIOVortex(), new WristIONeo());

        intakecommand = new IntakeFull(elevator, endEffector);
        l2command = new scorel2(elevator, endEffector);
        l3command = new scorel3(elevator, endEffector);
        l4command = new scorel4(elevator, endEffector);
        dealgifycommand = new dealgify(elevator, endEffector);
        drop = new OutakeClaw(endEffector);
        toOrigin = new PathOnTheFlyToPose(drive, new Pose2d());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(SwerveConstants.FrontLeft),
                new ModuleIOSim(SwerveConstants.FrontRight),
                new ModuleIOSim(SwerveConstants.BackLeft),
                new ModuleIOSim(SwerveConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement // ,
                // new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                // new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                // new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose),
                // new VisionIOPhotonVisionSim(camera4Name, robotToCamera4, drive::getPose)
                );

        endEffector = new EndEffector(new ClawIOVortex(), new WristIONeo());
        elevator = new Elevator(new ElevatorIONeo());

        climber = new Climber(new ClimberIOKraken());

        intakecommand = new IntakeFull(elevator, endEffector);
        l2command = new scorel2(elevator, endEffector);
        l3command = new scorel3(elevator, endEffector);
        l4command = new scorel4(elevator, endEffector);
        dealgifycommand = new dealgify(elevator, endEffector);
        drop = new OutakeClaw(endEffector);
        toOrigin = new PathOnTheFlyToPose(drive, new Pose2d());

        break;

      default:
        // Replayed robot, disable IO implementations

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(
                drive::addVisionMeasurement // ,
                // new VisionIO() {},
                // new VisionIO() {},
                // new VisionIO() {},
                // new VisionIO() {},
                // new VisionIO() {}
                );

        endEffector = new EndEffector(new ClawIOVortex(), new WristIONeo());

        elevator = new Elevator(new ElevatorIONeo());

        climber = new Climber(new ClimberIOKraken());

        intakecommand = new IntakeFull(elevator, endEffector);
        l2command = new scorel2(elevator, endEffector);
        l3command = new scorel3(elevator, endEffector);
        l4command = new scorel4(elevator, endEffector);
        dealgifycommand = new dealgify(elevator, endEffector);
        drop = new OutakeClaw(endEffector);
        toOrigin = new PathOnTheFlyToPose(drive, new Pose2d());

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive

    double reduction = Math.pow(elevator.getPercentRaised(), 2);
    double cappedreduction = 1; // Math.max(reduction, 0.70);

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> (-controller.getLeftY()),
            () -> (-controller.getLeftX()),
            () -> (-controller.getRightX())));

    elevator.setDefaultCommand(
        new InstantCommand(() -> elevator.moveElevator(copilot.getLeftY()), elevator));

    // climber.setDefaultCommand(
    //     new InstantCommand(() -> climber.driveClimber(copilot.getLeftY()), climber));

    copilot.a().onTrue(new InstantCommand(elevator::resetEncoder));

    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller.leftTrigger().onTrue(intakecommand);
    controller.leftBumper().onTrue(l2command);
    controller.x().onTrue(l3command);
    controller.a().onTrue(l4command);
    controller.rightBumper().onTrue(drop.withTimeout(1));
    // controller
    //     .rightTrigger()
    //     .whileTrue(
    //         AutoBuilder.pathfindToPose(new Pose2d(), new PathConstraints(null, null, null,
    // null)));

    // controller.rightTrigger().whileTrue(AutoBuilder.followPath(path));
    // controller.rightTrigger().whileTrue(toOrigin);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to ru
   *     <p>n in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("cauto1");
  }
}
