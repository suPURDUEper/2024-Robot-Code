// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot;

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

  // All canID constants go in here
  public static class CANIDs {
    public static final int kIntakeMotor = 0;
    public static final int kFeederMotor = 0;
    public static final int kAmpMotor = 0;
    public static final int kClimberMotor = 0;
    public static final int kClimber2Motor = 0;
    public static final int kElevatorMotor = 0;
    public static final int kTiltMotor = 0;
    public static final int kShooterRightMotor = 0;
    public static final int kShooterLeftMotor = 0;
  }

  // All DIOport constants go in here
  public static class DIOPorts {
    public static final int kFeederBreakBeam = 0;
    public static final int kAmpBreakBeam = 0;
    public static final int kTiltEncoder = 0;
  }

  public static class ElevatorConstants {
    public static final double kMaxTravelMeters = Units.inchesToMeters(21);
    public static final double kGearRatio = 9.0 / 1.0;
    public static final double kSprocketPitchDiameter = Units.inchesToMeters(1.751);
    public static final double kMetersPerRotation = kGearRatio * kSprocketPitchDiameter * Math.PI;
    public static final float kForwardSoftLimit = (float) Units.inchesToMeters(21);
    public static final float kReverseSoftLimit = 0;
  }

  public static class ClimberConstants {
    public static final double kPlanetaryGearRatio = (25.0 / 1.0);
    public static final double kSprocketGearRatio = (48.0 / 16.0);
    public static final double kAbsoluteEncoderOffset = 0;
    public static final float kForwardSoftLimit = 0;
    public static final float kReverseSoftLimit = 0;
  }

  public static class TiltConstants {
    public static final double kPlanetaryGearRatio = (9.0 / 1.0);
    public static final double kSectorGearRatio = (240.0 / 10.0);
    public static final double kGearRatio = kPlanetaryGearRatio * kSectorGearRatio;
    public static final int kForwardSoftLimit = 0;
    public static final int kReverseSoftLimit = 0;
    public static final double kAbsoluteEncoderOffset = 0;
    public static final double kEncoderPositionConversion = 0;
    public static final boolean kAbsoluteEncoderInverted = true;
    public static final int kStatorCurrentLimit = 0;

    public static final double kp = 0; // amps applied per rotation of error
    public static final double ki = 0; // leave at 0
    public static final double kd = 0; // amps applied per rps of error
    public static final double ks = 0; // minimum amount of amps to move arm
    public static final double kv = 0; // Probably leave at 0
    public static final double ka = 0; // Amps required to accelerate arm 1 rot/s^2
    public static final double kg = 0; // Amps required to hold arm level at 0 degrees
    public static final double profileKv = 0; // Kv of arm brushed motor model in volts/rps
    public static final double profileKa = 0; // Ka of arm brushed motor model in volts/(rps/s)
  }

  public static class ShooterConstants {
    public static final double kGearRatio = 1.0 / 2.0;
    public static final int kStatorCurrentLimit = 0;

    public static final double kp = 0; // amps applied per rotation of error
    public static final double ki = 0; // leave at 0
    public static final double kd = 0; // amps applied per rps of error
    public static final double ks = 0; // minimum amount of amps to move arm
    public static final double kv = 0; // Probably leave at 0
    public static final double ka = 0; // Amps required to accelerate arm 1 rot/s^2
    public static final double kg = 0; // Amps required to hold arm level at 0 degrees
  }
}
