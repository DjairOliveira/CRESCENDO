// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private CANSparkMax MotorNEO;
  private boolean CheckBox;

  @Override
  public void robotInit() 
  {
    MotorNEO = new CANSparkMax(6,MotorType.kBrushless);
    MotorNEO.setIdleMode(CANSparkMax.IdleMode.kCoast);

    SmartDashboard.putBoolean("CHECK 1", CheckBox);              // Cria o Botão no DashBoard
  }

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
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable smartDashboard = inst.getTable("SmartDashboard");
    boolean DashBTN1 = smartDashboard.getEntry("CHECK 1").getBoolean(false);

    if(DashBTN1==true) MotorNEO.setIdleMode(CANSparkMax.IdleMode.kBrake);
    else MotorNEO.setIdleMode(CANSparkMax.IdleMode.kCoast);

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
