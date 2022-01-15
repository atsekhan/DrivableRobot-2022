// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class FrankenbotExtendSolenoid extends CommandBase {
  /** Creates a new FrankenbotFlipSolenoid. */
  public FrankenbotExtendSolenoid() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pneumaticsSubsystem);

    Robot.simpleCSVLogger.writeData("ToggleSolenoidCommand");

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.simpleCSVLogger.writeData("TS initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.pneumaticsSubsystem.extendCylinder();
    Robot.simpleCSVLogger.writeData("TS executed");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
