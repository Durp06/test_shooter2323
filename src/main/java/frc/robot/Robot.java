// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.xml.crypto.Data;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

 //talon.setrotpos = 0 on a

public class Robot extends TimedRobot {

  public TalonFX wrist_motor = new TalonFX(10);
  public CANcoder wrist_encoder = new CANcoder(1);
  public DoubleArrayLogEntry doubleEntryz;
  public XboxController controller = new XboxController(0);

  private void configureDriveMotor() {
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveConfig.Slot0.kP = 20.0;
    driveConfig.Slot0.kI = 0;
    driveConfig.Slot0.kD = 0;

    wrist_motor.getConfigurator().apply(driveConfig);
}
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("PercentOut1", 0);
    DataLogManager.start();

    DataLog log = DataLogManager.getLog();

    doubleEntryz = new DoubleArrayLogEntry(log, "/my/doubleArray");
    configureDriveMotor();
  }

  public double getPercentOut1() {
    return NetworkTableInstance
      .getDefault()
      .getTable("/SmartDashboard")
      .getEntry("PercentOut1")
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
    SmartDashboard.putNumber("RPM 1", wrist_motor.getRotorVelocity().getValue() * 60);

    var request1 = new DutyCycleOut(controller.getLeftY());
    wrist_motor.setControl(request1);

    if (controller.getAButtonPressed())
      wrist_motor.setPosition(0);

    double[] vals = {wrist_motor.getPosition().getValue(), wrist_encoder.getPosition().getValue()};
    doubleEntryz.append(vals);
  }

  
}