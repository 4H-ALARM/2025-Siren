package frc.lib.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class RobotConstants {

  public static class IntakeConstants {
    public static final int tiltMotorID = 1;
    public static final int spinMotorID = 2;

    public static final Rotation2d inactiveAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d activeAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d holdAngle = Rotation2d.fromDegrees(0);
  }

  public static class ClimberConstants {

    public static final int climberMotorID = 50;
  }

  public static class EndEffectorConstants {

    public static int wristmotorID = 32;
    public static int clawmotorID = 33;
    public static int frontcanrange = 40;
    public static int intakecanrange = 41;

    public static final Rotation2d defaultrot = new Rotation2d().fromRotations(0.7);
    public static final Rotation2d intakerot = new Rotation2d().fromRotations(0.815);
    public static final Rotation2d L1rot = new Rotation2d().fromRotations(0.7);
    public static final Rotation2d L2rot = new Rotation2d().fromRotations(0.7);
    public static final Rotation2d L3rot = new Rotation2d().fromRotations(0.7);
    public static final Rotation2d L4rot = new Rotation2d().fromRotations(0.7);

    public static enum WristState {
      DEFAULT(defaultrot),
      INTAKE(intakerot),
      L1(L1rot),
      L2(L2rot),
      L3(L3rot),
      L4(L4rot);

      private final Rotation2d target;

      WristState(Rotation2d rotation2d) {
        this.target = rotation2d;
      }

      public Rotation2d getTargetRotation2d() {
        return target;
      }
    }
  }

  public static class ElevatorConstants {
    public static final int leadMotorID = 30;
    public static final int followerMotorID = 31;
    public static final int bottomlimitswitchID = 5;
    public static final int toplimitswitchID = 4;

    public static final Rotation2d defaultheight = new Rotation2d().fromRotations(0);
    public static final Rotation2d intakeheight = new Rotation2d().fromRotations(0);
    public static final Rotation2d L1height = new Rotation2d().fromRotations(0);
    public static final Rotation2d L2height = new Rotation2d().fromRotations(-15);
    public static final Rotation2d dealgifyheight = new Rotation2d().fromRotations(-(17.5));
    public static final Rotation2d L3height = new Rotation2d().fromRotations(-37.5);
    public static final Rotation2d L4height = new Rotation2d().fromRotations(-70);

    public static enum elevatorState {
      DEFAULT(defaultheight),
      INTAKE(intakeheight),
      L1(L1height),
      L2(L2height),
      DEALGIFY(dealgifyheight),
      L3(L3height),
      L4(L4height);

      private final Rotation2d target;

      elevatorState(Rotation2d rotation2d) {
        this.target = rotation2d;
      }

      public Rotation2d getTargetRotation2d() {
        return target;
      }
    }
  }
}
