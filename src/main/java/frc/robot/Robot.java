// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public TalonFX motor = new TalonFX(10);
  //public TalonFX motor2 = new TalonFX(11);


  private void configureDriveMotor() {
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveConfig.Slot0.kP = 20.0;
    driveConfig.Slot0.kI = 0;
    driveConfig.Slot0.kD = 0;

    motor.getConfigurator().apply(driveConfig);
    //motor2.getConfigurator().apply(driveConfig);
}
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("PercentOut1", 0);
    SmartDashboard.putNumber("PercentOut2", 0);

    configureDriveMotor();
  }

  public double getPercentOut1() {
    return NetworkTableInstance
      .getDefault()
      .getTable("/SmartDashboard")
      .getEntry("PercentOut1")
      .getDouble(0.0);
  }

    public double getPercentOut2() {
    return NetworkTableInstance
      .getDefault()
      .getTable("/SmartDashboard")
      .getEntry("PercentOut2")
      .getDouble(0.0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //SmartDashboard.putNumber("Speed", (motor.getRotorVelocity().getValue() / 2.0 ) * (2*Math.PI*2));
    //SmartDashboard.putNumber("RPS", motor.getRotorVelocity().getValue());
    SmartDashboard.putNumber("RPM 1", motor.getRotorVelocity().getValue() * 60);
    //SmartDashboard.putNumber("RPM 2", motor2.getRotorVelocity().getValue() * 60);


    var request1 = new DutyCycleOut(getPercentOut1());
    var request2 = new DutyCycleOut(getPercentOut2());

    motor.setControl(request1);
    //motor2.setControl(request2);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
