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

  public int canIDs[] = new int[] {8, 9, 10, 11};
  public TalonFX motors[] = new TalonFX[canIDs.length];


  private void configureDriveMotor(TalonFX y) {
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveConfig.Slot0.kP = 20.0;
    driveConfig.Slot0.kI = 0;
    driveConfig.Slot0.kD = 0;

    
    y.getConfigurator().apply(driveConfig);
}
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    for (int i = 0; i<canIDs.length; i++){
      motors[i] = new TalonFX(canIDs[i]);
       SmartDashboard.putNumber("PercentOut" + i, 0);
       configureDriveMotor(motors[i]);
    }
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
    for (int x = 0; x < motors.length; x++){
      var request = new DutyCycleOut(
      NetworkTableInstance
        .getDefault()
        .getTable("/SmartDashboard")
        .getEntry("PercentOut" + x)
        .getDouble(0.0)
        );
      motors[x].setControl(request);

      SmartDashboard.putNumber("RPM " + x, motors[x].getRotorVelocity().getValue() * 60);
      SmartDashboard.putNumber("Voltage", motors[x].getTorqueCurrent().getValue());
    }

  }
 

}
