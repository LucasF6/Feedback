// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Feedback.*;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeedbackSubsystem extends SubsystemBase {
  ArmSubsystem m_armSubsystem;
  ClawSubsystem m_clawSubsystem;
  TurretSubsystem m_turretSubsystem;
  VisionSubsystem m_visionSubsystem;
  WPI_Pigeon2 m_gyro;
  XboxController m_controller;
  PowerDistribution m_pdh;

  double m_currentToAccelerationRatio = 0;
  double m_netAcceleration = 0;

  /** Creates a new FeedbackSubsystem. */
  public FeedbackSubsystem(
    ArmSubsystem armSubsystem,
    ClawSubsystem clawSubsystem,
    TurretSubsystem turretSubsystem,
    VisionSubsystem visionSubsystem,
    WPI_Pigeon2 gyro,
    XboxController controller,
    PowerDistribution pdh
  ) {
    m_armSubsystem = armSubsystem;
    m_clawSubsystem = clawSubsystem;
    m_turretSubsystem = turretSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_gyro = gyro;
    m_controller = controller;
    m_pdh = pdh;
  }

  @Override
  public void periodic() {
    // Calculate net acceleration
    short[] xyzAcc = new short[3];
    double[] gravVec = new double[3];
    m_gyro.getBiasedAccelerometer(xyzAcc);
    m_gyro.getGravityVector(gravVec);
    double xAcc = xyzAcc[0] - GRAVITY * gravVec[0];
    double yAcc = xyzAcc[1] - GRAVITY * gravVec[1];
    double zAcc = xyzAcc[2] - GRAVITY * gravVec[2];
    m_netAcceleration = Math.sqrt(
      Math.pow(xAcc, 2) +
      Math.pow(yAcc, 2) +
      Math.pow(zAcc, 2)
    );

    // Calculates the ratio of the average current in drive motors to the robot's acceleration
    m_currentToAccelerationRatio = 0.25 * (
      m_pdh.getCurrent(DRIVE_POWER_CHANNEL[0]) +
      m_pdh.getCurrent(DRIVE_POWER_CHANNEL[1]) +
      m_pdh.getCurrent(DRIVE_POWER_CHANNEL[2]) +
      m_pdh.getCurrent(DRIVE_POWER_CHANNEL[3])
    ) / m_netAcceleration;

    // Set rumble of controller. Values of STRENGTH may vary depending on turret or claw.
    if (m_clawSubsystem.getStatus() || 
        m_currentToAccelerationRatio > CURRENT_RATIO_RUMBLE_THRESHOLD || 
        m_netAcceleration > ACCELERATION_RUMBLE_THRESHOLD) {
      m_controller.setRumble(RumbleType.kLeftRumble, STRENGTH);
      m_controller.setRumble(RumbleType.kRightRumble, STRENGTH);
    } else if (m_turretSubsystem.getLeftStatus()) {
      m_controller.setRumble(RumbleType.kLeftRumble, STRENGTH);
      m_controller.setRumble(RumbleType.kRightRumble, 0);
    } else if (m_turretSubsystem.getRightStatus()) {
      m_controller.setRumble(RumbleType.kLeftRumble, 0);
      m_controller.setRumble(RumbleType.kRightRumble, STRENGTH);
    } else {
      m_controller.setRumble(RumbleType.kLeftRumble, 0);
      m_controller.setRumble(RumbleType.kRightRumble, 0);
    }

    SmartDashboard.putNumber("Arm Angle Measurement", m_armSubsystem.getAngleMeasurement());
    SmartDashboard.putNumber("Arm Extension Measurement", m_armSubsystem.getExtensionMeasurement());
    SmartDashboard.putNumber("Turret Angle Measurement", m_turretSubsystem.getMeasurementDegrees());
    SmartDashboard.putNumber("Net Acceleration", m_netAcceleration);
    
    if (IS_TESTING) {
      SmartDashboard.putNumber("Gyro Yaw", m_gyro.getYaw());
      SmartDashboard.putNumber("Gyro Pitch", m_gyro.getPitch());
      SmartDashboard.putNumber("Gyro Roll", m_gyro.getRoll());
      SmartDashboard.putNumber("Target Yaw", m_visionSubsystem.getTargetYaw());
      SmartDashboard.putNumber("Target Pitch", m_visionSubsystem.getTargetPitch());
      SmartDashboard.putNumber("Current To Acceleration Ratio", m_currentToAccelerationRatio);
    }  
  }

  public boolean isHittingObstacle() {
    return m_currentToAccelerationRatio > CURRENT_RATIO_RUMBLE_THRESHOLD;
  }

  public boolean isOverAccelerationThreshold() {
    return m_netAcceleration > ACCELERATION_RUMBLE_THRESHOLD;
  }
}
