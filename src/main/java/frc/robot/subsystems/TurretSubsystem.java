// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getMeasurementDegrees() {
    return 0;
  }

  /** Return when left should rumble: too far to the left */
  public boolean getLeftStatus() {
    return false;
  }

  /** Return when right should rumble: too far to the right */
  public boolean getRightStatus() {
    return false;
  }
}
