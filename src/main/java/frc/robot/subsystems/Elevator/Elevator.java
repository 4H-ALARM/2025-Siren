// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.RobotConstants.ElevatorConstants.elevatorState;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIO elevator;

  private ElevatorIOInputsAutoLogged elevatorinputs;

  private elevatorState state;

  private TrapezoidProfile profile;
  private ElevatorFeedforward feedforward;
  private Timer profileTimer;
  private DigitalOutput output;

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;
    elevatorinputs = new ElevatorIOInputsAutoLogged();
    this.state = elevatorState.DEFAULT;

    profile = new TrapezoidProfile(new Constraints(5000, 50000));
    feedforward = new ElevatorFeedforward(0, 0.0, 0);
    profileTimer = new Timer();
    profileTimer.start();

    output = new DigitalOutput(4);
  }

  public void moveElevator(double input) {
    this.elevator.move(input);
  }

  public void setGoalState(elevatorState givenstate) {
    if (givenstate != this.state) {
      this.state = givenstate;
      profileTimer.reset();
      profileTimer.start();
    }
  }

  public elevatorState getGoalState() {
    return this.state;
  }

  public void stopElevator() {
    elevator.stopElevator();
    profileTimer.stop();
  }

  public ElevatorIO getElevatorInstance() {
    return this.elevator;
  }

  public void resetEncoder() {
    this.elevator.resetEncoder();
  }

  public double getPercentRaised() {
    return elevator.getPercentRaised();
  }

  @Override
  public void periodic() {
    if (profileTimer.isRunning()) {
      elevator.moveToPoint(
          Rotation2d.fromRotations(
              profile.calculate(
                          profileTimer.getTimestamp(),
                          new State(
                              this.elevator.getEncoder().getPosition(),
                              this.elevator.getEncoder().getVelocity()),
                          new State(this.state.getTargetRotation2d().getRotations(), 0))
                      .position
                  + feedforward.calculate(elevator.getEncoder().getVelocity())));
    }
    Logger.recordOutput("elevator/dio", output.get());
    elevator.updateInputs(elevatorinputs);
  }
}
