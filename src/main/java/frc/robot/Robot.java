// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveInterface;
import frc.robot.Constants.PigeonIMU;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.PotentiometerConstants;
import frc.robot.Constants.RobotDriveChassisConstants;
import frc.robot.Constants.RobotProperties;
import frc.robot.Constants.ShooterConstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * Robot on-board logging
   */
  public static final SimpleCSVLogger simpleCSVLogger = new SimpleCSVLogger();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // enable loggin if needed
    if (Constants.RobotProperties.robotLogging) {
      simpleCSVLogger.init(new String[] { "Module" }, new String[] { "Message" }); // start the logging; initialize the log file                                                                           // log file on the USB stick
    }

    // Configure Robot Settings based on the constants
    configureRobotSettings();


    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    // disable loggin if needed
    if (Constants.RobotProperties.robotLogging) {
      simpleCSVLogger.forceSync();
      simpleCSVLogger.close();
    }

  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // enable loggin if needed
    if (Constants.RobotProperties.robotLogging) {
      simpleCSVLogger.init(new String[] { "Module" }, new String[] { "Message" }); // start the logging; initialize the
                                                                                   // log file on the USB stick
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Update telemetry
    RobotContainer.smartDashboardSubsystem.updateDriveSubsystemTelemetry();
    RobotContainer.smartDashboardSubsystem.updatePDPValues();

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  private void configureRobotSettings() {

    switch (RobotProperties.robotModel) {
      case FRANKENBOT:

        // Subsystem Settings
        RobotProperties.isIMU = true;
        RobotProperties.isNaVX = true;
        RobotProperties.driveInterface = DriveInterface.ONESTICK;
        RobotProperties.isPneumatics = true;
        RobotProperties.isTEMPShooterTest = true;

        // Drivetrain settings
        DriveConstants.isInvertdGearBox = false;
        DriveConstants.leftMotorPortID = new int[] { 9 };
        DriveConstants.rightMotorPortID = new int[] { 10 };
        DriveConstants.kLeftEncoderPorts = new int[] { 9 };
        DriveConstants.kRightEncoderPorts = new int[] { 10 };
        DriveConstants.kLeftEncoderReversed = false;
        DriveConstants.kRightEncoderReversed = true;

        RobotDriveChassisConstants.wheelDiameter = 4;
        RobotDriveChassisConstants.encoderUnitsPerShaftRotation = 2048;
        RobotDriveChassisConstants.encoderGearReduction = 6.1;

        // Pneumatics
        PneumaticsConstants.compressorCANID = 0;
        PneumaticsConstants.SolenoidChannel = new int[] { 0, 7 };

        //Robot.simpleCSVLogger.writeData("******  ---- **** Subsystem Configured", "FRANKENBOT");
        break;
      case DEMOBOARD:

        // Subsystem Settings
        RobotProperties.isIMU = true;
        RobotProperties.isNaVX = false;
        RobotProperties.driveInterface = DriveInterface.ONESTICK;
        RobotProperties.isPneumatics = false;
        RobotProperties.isShooter = true;
        RobotProperties.isPotentiometer = true;

        // Drivetrain settings
        DriveConstants.isInvertdGearBox = false;
        DriveConstants.leftMotorPortID = new int[] { 1 };
        DriveConstants.rightMotorPortID = new int[] { 2 };
        DriveConstants.kLeftEncoderPorts = new int[] { 1 };
        DriveConstants.kRightEncoderPorts = new int[] { 2 };
        DriveConstants.kLeftEncoderReversed = false;
        DriveConstants.kRightEncoderReversed = true;

        // IMU
        PigeonIMU.pigeonIMUId = 3;

        // Shooter settings
        ShooterConstants.tiltMotorPortID = 4;

        //potentiometer
        PotentiometerConstants.PotentiometerPort = 3;
        

        Robot.simpleCSVLogger.writeData("Subsystem Configured", "DEMOBOARD");

        break;

      case C2022:

        // Subsystem Settings
        RobotProperties.isIMU = false;
        RobotProperties.isNaVX = false;
        RobotProperties.driveInterface = DriveInterface.SPLITSTICK;
        RobotProperties.isPneumatics = false;
        RobotProperties.isShooter = false;
        RobotProperties.isPotentiometer = false;

        // Drivetrain settings
        DriveConstants.isInvertdGearBox = true;
        DriveConstants.leftMotorPortID = new int[] { 3,4 };
        DriveConstants.rightMotorPortID = new int[] { 1,2 };
        DriveConstants.kLeftEncoderPorts = new int[] { 3,4 };
        DriveConstants.kRightEncoderPorts = new int[] { 1,2 };
        DriveConstants.kLeftEncoderReversed = false;
        DriveConstants.kRightEncoderReversed = true;

        // IMU

        // Shooter settings

        //potentiometer     

        Robot.simpleCSVLogger.writeData("Subsystem Configured", "C2022");

        break;

      default:

        // Subsystem Settings
        RobotProperties.isNaVX = true;
        RobotProperties.driveInterface = DriveInterface.SPLITSTICK;
        RobotProperties.isPneumatics = true;
    }

  }

}
