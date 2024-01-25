// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;

public class SpinnerExample extends ManipulatorBase {
  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double tolerance = 0;

  public SpinnerExample() {
    addMotors(new CANSparkMax(1, CANSparkMax.MotorType.kBrushless),
        new CANSparkMax(2, CANSparkMax.MotorType.kBrushless));

    setSpeedPID(kP, kI, kD, tolerance);
    setSpeedMultiplier(1);
  }

  public void shoot() {
    setTargetSpeed(360);
    getSpeedCommand().setUponTarget(new InstantCommand(() -> {

      // release the ring
      // after the ring is released, setUponTarget to:
      fullStop();
    }));
  }
}
