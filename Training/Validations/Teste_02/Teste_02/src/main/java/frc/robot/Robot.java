// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
  private CANSparkMax MSE;
  private CANSparkMax MSD;
  private CANSparkMax MIE;
  private CANSparkMax MID;

  private XboxController Joy1_EH;
  private XboxController Gat1_D;

  private XboxController BTN_1;

  private static final String DashDefault = "Default";
  private static final String DashOpt1 = "Option 01";
  private static final String DashOpt2 = "Option 02";
  private static final String DashOpt3 = "Option 03";
  private static final String DashOpt4 = "Option 04";

  private String m_autoSelected;

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //private static final String Button = "Cancel";

  @Override
  public void robotInit()
  {
    MSE = new CANSparkMax(4, CANSparkMax.MotorType.kBrushless);
    MSD = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
    MIE = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);
    MID = new CANSparkMax(8, CANSparkMax.MotorType.kBrushless);

    Joy1_EH = new XboxController(0);
    Gat1_D = new XboxController(0);
    BTN_1 = new XboxController(0);

    m_chooser.setDefaultOption("Default", DashDefault);
    m_chooser.addOption("Option 1", DashOpt1);
    m_chooser.addOption("Option 2", DashOpt2);
    m_chooser.addOption("Option 3", DashOpt3);
    m_chooser.addOption("Option 4", DashOpt4);

    SmartDashboard.putData("Escolha o Auto", m_chooser);
  }

  @Override
  public void robotPeriodic()
  {

  }

  @Override
  public void autonomousInit()
  {
  }

  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void teleopPeriodic()
  {
    double Gatilho = Gat1_D.getRawAxis(3);
    Boolean Invert = BTN_1.getAButton();

    SmartDashboard.putNumber("Valor do Gatilho:", Gatilho);
    SmartDashboard.putBoolean("Button = ", Invert);
    m_autoSelected = m_chooser.getSelected();

    if(m_autoSelected == "DashOpt3")
    {
      Gatilho = Gatilho/4;
      System.out.println(Gatilho);
    }
    if(m_autoSelected == "DashOpt1")
    {
      Gatilho = Gatilho/2;
      System.out.println(Gatilho);
    }

    if(Invert == true)
    {
      Gatilho = Gatilho*(-1);
    }

//System.out.println(Gatilho);

    MSE.set(Gatilho);
    MSD.set(Gatilho);
    MIE.set(Gatilho);
    MID.set(Gatilho);
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
