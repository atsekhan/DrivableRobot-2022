// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NetworkTablesSubsystem extends SubsystemBase {
  /** Creates a new NetworkTablesSubsystem. */

  private NetworkTableInstance ntInst;

  public NetworkTablesSubsystem() {
    ntInst = NetworkTableInstance.getDefault();
    ntInst.startClientTeam(Constants.TeamNumber);
  }

  public double getDouble(String tableName, String tableEntry, double defValue) {
    return ntInst.getTable(tableName).getEntry(tableEntry).getDouble(defValue);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
