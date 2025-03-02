// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.objectivepicker;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class objectivepicker {

  boolean[] reefL2 =
      new boolean[] {
        false, false, false, false, false, false, false, false, false, false, false, false
      };
  boolean[] reefL3 =
      new boolean[] {
        false, false, false, false, false, false, false, false, false, false, false, false
      };
  boolean[] reefL4 =
      new boolean[] {
        false, false, false, false, false, false, false, false, false, false, false, false
      };

  public objectivepicker() {
    SmartDashboard.putBooleanArray(
        "reefL2",
        new boolean[] {
          false, false, false, false, false, false, false, false, false, false, false, false
        });
    SmartDashboard.putBooleanArray(
        "reefL3",
        new boolean[] {
          false, false, false, false, false, false, false, false, false, false, false, false
        });
    SmartDashboard.putBooleanArray(
        "reefL4",
        new boolean[] {
          false, false, false, false, false, false, false, false, false, false, false, false
        });
  }

  public void periodic() {
    update();
  }

  //
  public void update() {
    reefL2 =
        SmartDashboard.getBooleanArray(
            "reefL2",
            new boolean[] {
              false, false, false, false, false, false, false, false, false, false, false, false
            });
    reefL3 =
        SmartDashboard.getBooleanArray(
            "reefL3",
            new boolean[] {
              false, false, false, false, false, false, false, false, false, false, false, false
            });
    reefL4 =
        SmartDashboard.getBooleanArray(
            "reefL4",
            new boolean[] {
              false, false, false, false, false, false, false, false, false, false, false, false
            });
  }

  public void scoredPiece(ReefPost post, Level level) {

    switch (level) {
      case L1:
        throw new Error("L1 not handled yet");
      case L2:
        putScoredValue(level, post, true);
        break;

      case L3:
        putScoredValue(level, post, true);
        break;

      case L4:
        putScoredValue(level, post, true);
        break;
    }
  }

  public void putScoredValue(Level name, ReefPost post, boolean value) {

    reefL2[post.getIndex()] = value;

    SmartDashboard.putBooleanArray(name.getName(), reefL2);
  }
  //

}

enum Level {
  L1("reefL1"),
  L2("reefL2"),
  L3("reefL3"),
  L4("reefL4");

  private String name;

  public String getName() {
    return name;
  }

  private Level(String name) {
    this.name = name;
  }
}

enum ReefPost {
  A(1),
  B(2),
  C(3),
  D(4),
  E(5),
  F(6),
  G(7),
  H(8),
  I(9),
  J(10),
  K(11),
  L(12);

  private int node;

  private ReefPost(int node) {
    this.node = node;
  }

  public int getIndex() {
    return node;
  }

  // public boolean[] set(boolean) {

  // }

}
