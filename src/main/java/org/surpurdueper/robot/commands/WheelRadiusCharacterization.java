// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.surpurdueper.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.util.LoggedTunableNumber;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.surpurdueper.robot.subsystems.drive.generated.TunerConstants;

public class WheelRadiusCharacterization extends Command {
  private final LoggedTunableNumber characterizationSpeed =
      new LoggedTunableNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 2);
  private final double driveRadius = TunerConstants.kDriveRadius;
  private final DoubleSupplier gyroYawRadsSupplier;

  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;

    private Direction(int value) {
      this.value = value;
    }
  }

  private final CommandSwerveDrivetrain drive;
  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelRadiusCharacterization(CommandSwerveDrivetrain drive, Direction omegaDirection) {
    this.drive = drive;
    this.omegaDirection = omegaDirection;
    gyroYawRadsSupplier = () -> drive.getGyroYawRads();
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = drive.getWheelAngularPositionsRadians();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    drive.runWheelRadiusCharacterization(
        omegaLimiter.calculate(omegaDirection.value * characterizationSpeed.get()));

    // Get yaw and wheel positions
    accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = drive.getWheelAngularPositionsRadians();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    SmartDashboard.putNumber("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
    SmartDashboard.putNumber("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
    SmartDashboard.putNumber(
        "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
        Units.metersToInches(currentEffectiveWheelRadius));
  }

  @Override
  public void end(boolean interrupted) {
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}
