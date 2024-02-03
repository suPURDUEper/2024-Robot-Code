// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class ElevatorConstants {
    public static final double kMaxTravelMeters = Units.inchesToMeters(21);
    public static final double kGearRatio = 9.0 / 1.0;
    public static final double kSprocketPitchDiameter = Units.inchesToMeters(1.751);
    public static final double kMetersPerRotation = kGearRatio * kSprocketPitchDiameter * Math.PI;
    public static final float kForwardSoftLimit = (float) Units.inchesToMeters(21);
    public static final float kReverseSoftLimit = 0;
  }

  public static class ClimberConstants {
    public static final double kMaxTravelMeters = Units.inchesToMeters(21);
    public static final double kPlanetaryGearRatio = (25.0 / 1.0);
    public static final double kSprocketGearRatio = (48.0 / 16.0);
    public static final float kForwardSoftLimit = 0;
    public static final float kReverseSoftLimit = 0;
  }

  public static class ShooterAngleConstants {
    public static final double kPlanetaryGearRatio = (9.0 / 1.0);
    public static final double kSectorGearRatio = (240.0 / 10.0);
    public static final double kAbsoluteEncoderOffset = 0;
    public static final float kForwardSoftLimit = ;
    public static final float kReverseSoftLimit = 0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CANIDs {
    public static final int kIntakeMotor = 0;
    public static final int kFeederMotor = 0;
    public static final int kAmpMotor = 0;
    public static final int kClimberMotor = 0;
    public static final int kTiltMotor = 0;
    public static final int kElevatorMotor = 0;
    public static final int kClimber2Motor = 0;
  }

  public static class DIOPorts {
    public static final int kFeederBreakBeam = 0;
    public static final int kAmpBreakBeam = 0;
  }
}
