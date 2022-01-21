// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {

    // This subsystem needs to be instantiated after all device-related ones
    updateDriveSubsystemTelemetry(); // initial update for the drive subsystem

  }

  // PDP telemetry
  public void updatePDPValues() {
    SmartDashboard.putNumber("PDP Voltage", RobotContainer.pdpSubsystem.getvoltage());
    SmartDashboard.putNumber("PDP Current", RobotContainer.pdpSubsystem.getcurrent());
    SmartDashboard.putNumber("PDP power", RobotContainer.pdpSubsystem.getpower());
  }

  public void updateIMUValues() {

    SmartDashboard.putString("IMU-Y-P-R",
        String.format("%12.6f", RobotContainer.imuSubsystem.getYaw()) + "  "
            + String.format("%12.6f", RobotContainer.imuSubsystem.getPitch()) + "  "
            + String.format("%12.6f", RobotContainer.imuSubsystem.getRoll()));

  }

  public void updateDriveSubsystemTelemetry() {
    SmartDashboard.putNumber("Left Encoder Value", RobotContainer.driveSubsystem.getLeftEncoder());
    SmartDashboard.putNumber("Left Encoder Speed", RobotContainer.driveSubsystem.getLeftEncoderSpeed());
    SmartDashboard.putNumber("Right Encoder Value", RobotContainer.driveSubsystem.getRightEncoder());
    SmartDashboard.putNumber("Right Encoder Speed", RobotContainer.driveSubsystem.getRightEncoderSpeed());
  }

  public void updateAllDisplays() {
    updateIMUValues();
    updateDriveSubsystemTelemetry();
  }

  // Trajectory/kinematic driving update; updated from NavigationControlSubsystem
  public void updateMeterPrint(double left, double right) {
    SmartDashboard.putNumber("left m", left);
    SmartDashboard.putNumber("right m", right);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}
