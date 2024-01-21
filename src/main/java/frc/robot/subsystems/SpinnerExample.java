// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

// import frc.robot.Constants;
// import frc.robot.Constants.DeviceConstants;
// import frc.robot.Constants.DriveConstants;
import frc.robot.SyncedLibraries.SystemBases.ManipulatorBase;

public class SpinnerExample extends ManipulatorBase {
  /** Creates a new SpinnerExample. */
  private static final double kP = 0;
  private static final double kI = 0;
  private static final double kD = 0;
  private static final double tolerance = 0;
  public SpinnerExample() {
    addMotors(new CANSparkMax(1, CANSparkMax.MotorType.kBrushless),
        new CANSparkMax(2, CANSparkMax.MotorType.kBrushless));
    setSpeedPID(kP, kI, kD, tolerance);
    setSpeedMultiplier(1);
    setPositionPID(kP, kI, kD, tolerance);
    setPositionMultiplier(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void home() {
    // TODO Auto-generated method stub
  }

  public void shoot() {
    moveToPosition(360);
  }
}
