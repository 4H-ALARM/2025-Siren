package frc.robot.commands.GroundIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.groundintake.GroundIntake;

public class groundIntakeOut extends Command {
  private GroundIntake groundIntake;

  public groundIntakeOut(GroundIntake w) {
    this.groundIntake = w;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    groundIntake.setAngle(Rotation2d.fromRotations(1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    groundIntake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
