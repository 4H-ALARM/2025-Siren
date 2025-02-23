// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  ClimberIO climber;

  ClimberIOInputsAutoLogged inputs;

  public Climber(ClimberIO climberimpl) {
    this.climber = climberimpl;

    inputs = new ClimberIOInputsAutoLogged();
  }

  @Override
  public void periodic() {

    climber.updateInputs(inputs);
  }
}
