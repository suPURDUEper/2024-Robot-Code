// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot;

import org.littletonrobotics.util.FieldConstants;

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
  public static final double kBumperWidthInches = 3.25;
  public static final double kBumperWidthMeters = Units.inchesToMeters(kBumperWidthInches);
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
    public static final int kClimberEncoder = 9;
    public static final int kClimberLimit1 = 4;
    public static final int kClimberLimit2 = 5;
  }

  public static class AmpConstants {

    public static final double kStatorCurrentLimit = 40;
    public static final double kSupplyCurrentLimit = 40;
    public static final double kSupplyCurrentLimitThreshold = 60;
    public static final double kSupplyTimeThreshold = 0.5;
    public static final double kLoadVoltage = 12;
    public static final double kTrapLoadVoltage = -1;
    public static final double kScoreVoltage = 12;
  }

  public static class ElevatorConstants {
    public static final double kMaxTravelMeters = Units.inchesToMeters(21);
    public static final double kGearRatio = 9.0 / 1.0;
    public static final double kSprocketPitchDiameter = Units.inchesToMeters(1.751);
    public static final double kMetersPerRotation = kGearRatio / (kSprocketPitchDiameter * Math.PI);
    public static final float kForwardSoftLimit = (float) Units.inchesToMeters(20.75);
    public static final float kReverseSoftLimit = 0;
    public static final double kClimbHeight = Units.inchesToMeters(20.5);
    public static final double kAfterClimbHeight = Units.inchesToMeters(14);
    public static final double kAmpScoreHeight = Units.inchesToMeters(17.75);

    // Feedforward gains from sysId
    public static final double kg = 0.31678; // volts
    public static final double kv = 8.0; // V*s/m
    public static final double ka = 0.03; // V*s^2/m
    public static final double kPositionTolerance = Units.inchesToMeters(0.25);
    public static final double kStatorCurrentLimit = 40;
    public static final double kp = 12.762;
    public static final double ki = 0;
    public static final double kd = 0;
    public static final double profileKv = 10.0;
    public static final double profileKa = 1.0;
  }

  public static class ClimberConstants {
    public static final double kPlanetaryGearRatio = (125.0 / 1.0);
    public static final double kSprocketGearRatio = (34.0 / 10.0);
    public static final double kAbsoluteEncoderOffset = Units.degreesToRotations(27.050374);
    public static final double kForwardSoftLimit = Units.degreesToRotations(70);
    public static final double kReverseSoftLimit = Units.degreesToRotations(-20);
    public static final double kp = 67.136;
    public static final double ki = 0;
    public static final double kd = 59.544;
    public static final double ks = 0.81355;
    public static final double kv = 37.222;
    public static final double ka = 0.76096;
    public static final double kg = 0.78433;
    public static final double profileKv = 40;
    public static final double profileKa = 1;
    public static final boolean kAbsoluteEncoderInverted = true;
    public static final double kStatorCurrentLimit = 100;
    public static final double kGearRatio = kPlanetaryGearRatio * kSprocketGearRatio;
    public static final double kPositionTolerance = 0;
    public static final double kClimbPosition = Units.degreesToRotations(39.814);
  }

  public static class TiltConstants {
    public static final double kPlanetaryGearRatio = (25.0 / 1.0);
    public static final double kSectorGearRatio = (240.0 / 10.0);
    public static final double kGearRatio = kPlanetaryGearRatio * kSectorGearRatio / 1.07015;
    public static final double kForwardSoftLimit = Units.degreesToRotations(62.5);
    public static final double kReverseSoftLimit = Units.degreesToRotations(26.2);
    public static final double kAbsoluteEncoderOffset =
        (Units.degreesToRotations(-1.058593) - Units.degreesToRotations(90.5)) + 1;
    public static final double kEncoderPositionConversion = 0;
    public static final boolean kAbsoluteEncoderInverted = false;
    public static final int kStatorCurrentLimit = 14;
    public static final double kWallShot = Units.degreesToRotations(54.5);
    public static final double kStageShot = Units.degreesToRotations(29.25);

    public static final double kp = 2000.0; // amps applied per rotation of error
    public static final double ki = 0; // leave at 0
    public static final double kd = 92.668; // amps applied per rps of error
    public static final double ks = 0.29174; // minimum amount of amps to move arm
    public static final double kv = 65.615; // Probably leave at 0
    public static final double ka = 0.70719; // Amps required to accelerate arm 1 rot/s^2
    public static final double kg = 0.33643; // Amps required to hold arm level at 0 degrees
    public static final double profileKv = 70; // Kv of arm brushed motor model in volts/rps
    public static final double profileKa =
        0.70719; // Ka of arm brushed motor model in volts/(rps/s)

    public static final double kPositionTolerance = Units.degreesToRotations(0.6);
    public static final double kPodiumShot = Units.degreesToRotations(30.0);
    public static final double kSubwooferShot = Units.degreesToRotations(51.0);
    public static final double kMaxAutoAim = 0.0;
    public static final double kIntakeAngle = Units.degreesToRotations(40.0);
    public static final double kAmpHandOff = Units.degreesToRotations(48.0);
    public static final double kSafeElevator = Units.degreesToRotations(54);
    public static final double kHardStopPosition = Units.degreesToRotations(18.17);
  }

  public static class ShooterConstants {
    public static final double kGearRatio = 24.0 / 36.0;
    public static final int kStatorCurrentLimit = 80;

    public static final double kp = 10.0; // 0.00015287; // amps applied per rotation of error
    public static final double ki = 0; // leave at 0
    public static final double kd = 0; // amps applied per rps of error
    public static final double ks = 10.0; // minimum amount of amps to move arm
    public static final double kv = 0.1; // Probably leave at 0
    public static final double ka = 0.0; // Amps required to accelerate arm 1 rot/s^2
    public static final double kShooterRpsTolerance = 10;

    public static final double kLeftShooterSpeedRps = 6000 / 60.0;
    public static final double kRightShooterSpeedRps = 3000 / 60.0;
    public static final double kLeftShooterIdleRps = 1500 / 60.0;
    public static final double kRightShooterIdleRps = 1500 / 60.0;
    public static final double kLeftShooterAmpRps = 2000 / 60.0;
    public static final double kRightShooterAmpRps = 2000 / 60.0;
  }

  public static class LookupTables {
    // Key is shooter angle (in rotations). Value is minimum elevator height such that the shot
    // clears the shooter (in meters from bottom hardstop)
    public static final InterpolatingDoubleTreeMap elevatorShooterClearance =
        new InterpolatingDoubleTreeMap();

    static {
      elevatorShooterClearance.put(Units.degreesToRotations(32.821), Units.inchesToMeters(0.000));
      elevatorShooterClearance.put(Units.degreesToRotations(34.000), Units.inchesToMeters(2.037));
      elevatorShooterClearance.put(Units.degreesToRotations(36.000), Units.inchesToMeters(3.835));
      elevatorShooterClearance.put(Units.degreesToRotations(38.000), Units.inchesToMeters(4.984));
      elevatorShooterClearance.put(Units.degreesToRotations(40.000), Units.inchesToMeters(6.451));
      elevatorShooterClearance.put(Units.degreesToRotations(42.000), Units.inchesToMeters(8.324));
      elevatorShooterClearance.put(Units.degreesToRotations(44.000), Units.inchesToMeters(12.640));
      elevatorShooterClearance.put(Units.degreesToRotations(49.5), Units.inchesToMeters(20));
      elevatorShooterClearance.put(Units.degreesToRotations(50.5), Units.inchesToMeters(20.75));
    }

    public static final InterpolatingDoubleTreeMap distanceToShooterAngle =
        new InterpolatingDoubleTreeMap();

    private static void addPointToDistanceToShooterAngle(
        double distanceInches, double angleDegrees) {
      distanceToShooterAngle.put(
          Units.inchesToMeters(distanceInches) + FieldConstants.subwooferToSpeakerCenter, 
          Units.degreesToRotations(angleDegrees));
    }

    static {
      addPointToDistanceToShooterAngle(00, 51.0);
      addPointToDistanceToShooterAngle(12, 48.5);
      addPointToDistanceToShooterAngle(24, 42.5);
      addPointToDistanceToShooterAngle(36, 38.5);
      addPointToDistanceToShooterAngle(48, 35.5);
      addPointToDistanceToShooterAngle(60, 33.5);
      addPointToDistanceToShooterAngle(72, 31.5);
      addPointToDistanceToShooterAngle(84, 29.5);
      addPointToDistanceToShooterAngle(96, 27);
      addPointToDistanceToShooterAngle(108, 25.5);
      addPointToDistanceToShooterAngle(120, 24.5);
      addPointToDistanceToShooterAngle(132, 24);
      addPointToDistanceToShooterAngle(144, 23);
      addPointToDistanceToShooterAngle(156, 22);
    }

    public static final InterpolatingDoubleTreeMap limelightTyToDistance = new InterpolatingDoubleTreeMap();

    static {
      limelightTyToDistance.put(25.54, Units.inchesToMeters(14.0));
      limelightTyToDistance.put(21.0, Units.inchesToMeters(24));
      limelightTyToDistance.put(16.65, Units.inchesToMeters(34));
      limelightTyToDistance.put(13.75, Units.inchesToMeters(45));
      limelightTyToDistance.put(9.35, Units.inchesToMeters(61));
      limelightTyToDistance.put(5.08, Units.inchesToMeters(72.5));
      limelightTyToDistance.put(6.92, Units.inchesToMeters(83));
      limelightTyToDistance.put(3.74, Units.inchesToMeters(92.5));
      limelightTyToDistance.put(2.35, Units.inchesToMeters(102.5));
      limelightTyToDistance.put(0.95, Units.inchesToMeters(116.5));
      limelightTyToDistance.put(0.26, Units.inchesToMeters(123));
      limelightTyToDistance.put(-0.75, Units.inchesToMeters(136));
      limelightTyToDistance.put(-2.50, Units.inchesToMeters(158));
    }
  }
}
