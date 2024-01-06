package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return null;
  }

  private void configureBindings() {}
}
