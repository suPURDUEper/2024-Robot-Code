// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static boolean tuningMode = true;

  public static final double kRobotLengthInches = 30.5;
  public static final double kRobotWidthInches = 29.5;
  public static final double kBumperWidthInches = 3.375;
  public static final double kBumperToRobotCenter =
      Units.inchesToMeters(kRobotLengthInches / 2.0) + Units.inchesToMeters(kBumperWidthInches);

  // All canID constants go in here
  public static class CANIDs {
    public static final int kIntakeMotor = 10;
    public static final int kFeederMotor = 11;
    public static final int kAmpMotor = 15;
    public static final int kClimberMotor = 17;
    public static final int kClimber2Motor = 18;
    public static final int kElevatorMotor = 16;
    public static final int kTiltMotor = 14;
    public static final int kShooterRightMotor = 13;
    public static final int kShooterLeftMotor = 12;
  }

  // All DIOport constants go in here
  public static class DIOPorts {
    public static final int kFeederBreakBeam1 = 8;
    public static final int kFeederBreakBeam2 = 7;
    public static final int kAmpBreakBeam = 6;
    public static final int kTiltEncoder = 9;
  }

  public static class AmpConstants {

    public static final double kStatorCurrentLimit = 40;
    public static final double kSupplyCurrentLimit = 40;
    public static final double kSupplyCurrentLimitThreshold = 60;
    public static final double kSupplyTimeThreshold = 0.5;
    public static final double kLoadVoltage = 12;
    public static final double kScoreVoltage = 12;
  }

  public static class ElevatorConstants {
    public static final double kMaxTravelMeters = Units.inchesToMeters(21);
    public static final double kGearRatio = 9.0 / 1.0;
    public static final double kSprocketPitchDiameter = Units.inchesToMeters(1.751);
    public static final double kMetersPerRotation = kGearRatio / (kSprocketPitchDiameter * Math.PI);
    public static final float kForwardSoftLimit = (float) Units.inchesToMeters(20.75);
    public static final float kReverseSoftLimit = 0;
    public static final double kAmpScoreHeight = Units.inchesToMeters(18.0);

    // Feedforward gains from sysId
    public static final double kg = 0.31678; // volts
    public static final double kv = 8.0; // V*s/m
    public static final double ka = 0.03; // V*s^2/m
    public static final double kPositionTolerance = Units.inchesToMeters(0.1);
    public static final double kStatorCurrentLimit = 40;
    public static final double kp = 12.762;
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double profileKv = 10.0;
    public static final double profileKa = 1.0;
  }

  public static class ClimberConstants {
    public static final double kPlanetaryGearRatio = (25.0 / 1.0);
    public static final double kSprocketGearRatio = (48.0 / 16.0);
    public static final double kAbsoluteEncoderOffset = 0;
    public static final float kForwardSoftLimit = 0;
    public static final float kReverseSoftLimit = 0;
  }

  public static class TiltConstants {
    public static final double kPlanetaryGearRatio = (25.0 / 1.0);
    public static final double kSectorGearRatio = (240.0 / 10.0);
    public static final double kGearRatio = kPlanetaryGearRatio * kSectorGearRatio;
    public static final double kForwardSoftLimit = Units.degreesToRotations(78.5);
    public static final double kReverseSoftLimit = Units.degreesToRotations(26.2);
    public static final double kAbsoluteEncoderOffset =
        (Units.degreesToRotations(8.079120) - 0.25) + 1;
    public static final double kEncoderPositionConversion = 0;
    public static final boolean kAbsoluteEncoderInverted = false;
    public static final int kStatorCurrentLimit = 60;

    public static final double kp = 720.0; // amps applied per rotation of error
    public static final double ki = 0; // leave at 0
    public static final double kd = 92.668; // amps applied per rps of error
    public static final double ks = 0.29174; // minimum amount of amps to move arm
    public static final double kv = 65.615; // Probably leave at 0
    public static final double ka = 0.70719; // Amps required to accelerate arm 1 rot/s^2
    public static final double kg = 0.33643; // Amps required to hold arm level at 0 degrees
    public static final double profileKv = 80; // Kv of arm brushed motor model in volts/rps
    public static final double profileKa =
        0.70719; // Ka of arm brushed motor model in volts/(rps/s)

    public static final double kPositionTolerance = Units.degreesToRotations(0.5);
    public static final double kPodiumShot = Units.degreesToRotations(30.0);
    public static final double kSubwooferShot = Units.degreesToRotations(61.0);
    public static final double kMaxAutoAim = 0.0;
    public static final double kIntakeAngle = Units.degreesToRotations(42.5);
    public static final double kAmpHandOff = Units.degreesToRotations(48.0);
    public static final double kSafeElevator = Units.degreesToRotations(54);
  }

  public static class ShooterConstants {
    public static final double kGearRatio = 1.0 / 2.0;
    public static final int kStatorCurrentLimit = 80;

    public static final double kp = 0.35; // 0.00015287; // amps applied per rotation of error
    public static final double ki = 0; // leave at 0
    public static final double kd = 0; // amps applied per rps of error
    public static final double ks = 0.25724; // minimum amount of amps to move arm
    public static final double kv = 0.067409; // Probably leave at 0
    public static final double ka = 0.022749; // Amps required to accelerate arm 1 rot/s^2
    public static final double kShooterRpsTolerance = 10;

    public static final double kLeftShooterSpeedRps = 6000 / 60.0;
    public static final double kRightShooterSpeedRps = 3000 / 60.0;
  }

  public static class LookupTables {
    // Key is shooter angle (in degrees). Value is minimum elevator height such that the shot clears
    // the shooter
    // (in inches from bottom hardstop)
    public static final InterpolatingDoubleTreeMap elevatorShooterClearance =
        new InterpolatingDoubleTreeMap();

    static {
      elevatorShooterClearance.put(34.821, 0.000); // Max angle with elevator all the way down
      elevatorShooterClearance.put(36.000, 1.037);
      elevatorShooterClearance.put(38.000, 2.835);
      elevatorShooterClearance.put(40.000, 3.984);
      elevatorShooterClearance.put(42.000, 5.451);
      elevatorShooterClearance.put(44.000, 7.324);
      elevatorShooterClearance.put(46.000, 9.640);
      elevatorShooterClearance.put(48.000, 12.291);
      elevatorShooterClearance.put(50.000, 15.186);
      elevatorShooterClearance.put(
          51.69, 17.725); // Max angle we can shoot at before jumping to 60 degree shot
    }

    public static final InterpolatingDoubleTreeMap distanceToShooterAngle =
        new InterpolatingDoubleTreeMap();

    static {
    }
  }
}
