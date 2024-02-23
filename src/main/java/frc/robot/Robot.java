// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

 //talon.setrotpos = 0 on a

public class Robot extends TimedRobot {

  public TalonFX wrist_motor = new TalonFX(16, "SuperStructureBus");
  public CANcoder wrist_encoder = new CANcoder(26, "SuperStructureBus");
  public DoubleArrayLogEntry doubleEntryz;
  public XboxController controller = new XboxController(0);
  public boolean logStarted = false;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
  }

  private double encToMotor(double motor) {
    return 0.00717*motor+0.328;
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
    var request1 = new DutyCycleOut(controller.getLeftY() * 0.4);
    wrist_motor.setControl(request1);

    if (controller.getAButtonPressed())
      wrist_motor.setPosition(0);

    if (controller.getBButtonPressed() && !logStarted) {
      doubleEntryz = new DoubleArrayLogEntry(
        DataLogManager.getLog(),
        "/my/doubleArray"
      );
      logStarted = true;
    }

    double[] vals = {wrist_motor.getPosition().getValue(), wrist_encoder.getPosition().getValue()};

    SmartDashboard.putNumber("motorRots", vals[0]);
    SmartDashboard.putNumber("encoderDeg", vals[1]);
    SmartDashboard.putNumber("estEncoder", encToMotor(vals[0]));

    if (!logStarted) {
      return;
    }

    doubleEntryz.append(vals);
  }

  
}