// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  CANSparkMax M1 = new CANSparkMax(30, MotorType.kBrushless);
  CANSparkMax M2 = new CANSparkMax(31, MotorType.kBrushless);
  CANSparkMax M3 = new CANSparkMax(32, MotorType.kBrushless);
  CANSparkMax M4 = new CANSparkMax(33, MotorType.kBrushless);

  XboxController controle = new XboxController(0);
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() 
  {
    M1.set(controle.getRightTriggerAxis()*(-1));
    M2.set(controle.getRightTriggerAxis());
    M3.set(controle.getRightTriggerAxis()*(-1));
    M4.set(controle.getRightTriggerAxis());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
