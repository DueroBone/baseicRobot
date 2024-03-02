package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SyncedLibraries.BasicFunctions;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return null;
  }

  private void configureBindings() {
    // Robot.Primary.LeftBumper.get().or(Robot.Primary.RightBumper.get())
    //     .onTrue(new InstantCommand(() -> {
    //       Robot.DriveTrain.doSlowMode(true);
    //     }))
    //     .onFalse(new InstantCommand(() -> {
    //       Robot.DriveTrain.doSlowMode(false);
    //     }));

    // Robot.Primary.RightTrigger.get()
    //     .onTrue(new InstantCommand(() -> {
    //       Robot.DriveTrain.setBrakeMode(true);
    //     }))
    //     .onFalse(new InstantCommand(() -> {
    //       Robot.DriveTrain.setBrakeMode(false);
    //     }));

    // Robot.Primary.LeftTrigger.get()
    //     .onTrue(new InstantCommand(() -> {
    //       Robot.DriveTrain.setBrakeMode(true);
    //     }))
    //     .onFalse(new InstantCommand(() -> {
    //       Robot.DriveTrain.setBrakeMode(false);
    //     }));
    // Robot.Primary.X
    //     .onTrue(new InstantCommand(() -> {
    //       Robot.Limelight.alignTag();
    //     }));
    Robot.Zero.A.get().onTrue(new InstantCommand(() -> {
      CommandScheduler.getInstance().cancel(Robot.teleDriveCommandBase);
      Robot.DriveTrain.doTankDrive(1, 1);
    }));
    Robot.Zero.B.get().onTrue(new InstantCommand(() -> {
      CommandScheduler.getInstance().schedule(Robot.teleDriveCommandBase);
    }));

    Robot.Zero.LeftBumper.get()
            .and(Robot.Zero.LeftTrigger.get())
            .and(Robot.Zero.LeftBumper.get())
            .and(Robot.Zero.RightBumper.get())
            .and(Robot.Zero.RightTrigger.get())
            .and(Robot.Zero.LeftStickPress.get())
            .and(Robot.Zero.RightStickPress.get())
            .onTrue(new InstantCommand(() -> BasicFunctions.KILLIT(Robot.DriveTrain)));
  }
}
