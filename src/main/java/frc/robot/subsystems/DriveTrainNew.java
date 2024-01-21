package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.SyncedLibraries.SystemBases.DriveTrainBase;

public class DriveTrainNew extends DriveTrainBase {

  public DriveTrainNew(CANSparkMax[] leftDriveMotors, CANSparkMax[] rightDriveMotors, PneumaticsModuleType moduleType,
      int[] GearChangerPorts, boolean kSkipGyro, double maxSpeed, int driveAmpsMax, int drivingRamp,
      double wheelDiameter, double pulsesPerRevolution, boolean disableShifter) {
        
    super(leftDriveMotors, rightDriveMotors, moduleType, GearChangerPorts, kSkipGyro, maxSpeed, driveAmpsMax, drivingRamp,
        wheelDiameter, pulsesPerRevolution, disableShifter);
    //TODO Auto-generated constructor stub
  }
  
}
