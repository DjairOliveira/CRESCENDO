// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import swervelib.encoders.CANCoderSwerve;

import com.ctre.phoenix6.hardware.CANcoder;

public class Robot extends TimedRobot 
{
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private CANcoder BL, BR, FL, FR;


  @Override
  public void robotInit() 
  {
    m_robotContainer = new RobotContainer();

    BL = new CANcoder(9);
    BR = new CANcoder(12);
    FL = new CANcoder(10);
    FR = new CANcoder(11);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("CANCODER_BL", BL.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("CANCODER_BR", BR.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("CANCODER_FL", FL.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("CANCODER_FR", FR.getAbsolutePosition().getValue());

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() 
  {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() 
  {

  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() 
  {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
