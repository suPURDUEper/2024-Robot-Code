// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.littletonrobotics.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoggedDashboardNumber {
  private final String key;
  private double defaultValue;
  private double value;

  /**
   * Creates a new LoggedDashboardNumber, for handling a number input sent via NetworkTables.
   *
   * @param key The key for the number, published to "/SmartDashboard/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   */
  public LoggedDashboardNumber(String key) {
    this(key, 0.0);
  }

  /**
   * Creates a new LoggedDashboardNumber, for handling a number input sent via NetworkTables.
   *
   * @param key The key for the number, published to "/SmartDashboard/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedDashboardNumber(String key, double defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue;
    SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
  }

  /** Updates the default value, which is used if no value in NT is found. */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
  }

  /**
   * Publishes a new value. Note that the value will not be returned by {@link #get()} until the
   * next cycle.
   */
  public void set(double value) {
    SmartDashboard.putNumber(key, value);
  }

  /** Returns the current value. */
  public double get() {
    value = SmartDashboard.getNumber(key, defaultValue);
    return value;
  }
}
