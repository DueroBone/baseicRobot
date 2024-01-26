package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return null;
  }

  private void configureBindings() {
    Robot.Primary.LeftBumper.or(Robot.Primary.RightBumper)
        .onTrue(new InstantCommand(() -> {
          Robot.DriveTrain.doSlowMode(true);
        }))
        .onFalse(new InstantCommand(() -> {
          Robot.DriveTrain.doSlowMode(false);
        }));

    Robot.Primary.RightTrigger
        .onTrue(new InstantCommand(() -> {
          Robot.DriveTrain.setBrakeMode(true);
        }))
        .onFalse(new InstantCommand(() -> {
          Robot.DriveTrain.setBrakeMode(false);
        }));

    Robot.Primary.LeftTrigger
        .onTrue(new InstantCommand(() -> {
          Robot.DriveTrain.setBrakeMode(true);
        }))
        .onFalse(new InstantCommand(() -> {
          Robot.DriveTrain.setBrakeMode(false);
        }));
    // Robot.Primary.X
    //     .onTrue(new InstantCommand(() -> {
    //       Robot.Limelight.alignTag();
    //     }));
  }
}
