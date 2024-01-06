package frc.robot;

public class BasicFunctions {
  public static double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return (input - (Math.abs(input) / input * deadband)) / (1 - deadband);
    }
  }
}
