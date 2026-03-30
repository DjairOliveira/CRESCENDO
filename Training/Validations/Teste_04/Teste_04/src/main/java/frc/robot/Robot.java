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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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

   private CANSparkMax MSE;
   private CANSparkMax MSD;
   private CANSparkMax MIE;
   private CANSparkMax MID;

   private XboxController Gat1_D;
   private XboxController BTN_1;
 
   private static final String DashDefault  = "0";
   private static final String DashOpt1     = "1";
   private static final String DashOpt2     = "2";
   private static final String DashOpt3     = "3";
   private static final String DashOpt4     = "4";
 
   private String m_autoSelected;
   private boolean CheckBox;

   private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit()
  {
    MSE = new CANSparkMax(4, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor
    MSD = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor
    MIE = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor
    MID = new CANSparkMax(8, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor

    Gat1_D = new XboxController(0);
    BTN_1 = new XboxController(0);

    m_chooser.setDefaultOption("MODO 1", DashDefault);      // Cria o botão principal
    m_chooser.addOption("MODO 2", DashOpt1);                // Adiciona botões a matriz
    m_chooser.addOption("MODO 3", DashOpt2);                // Adiciona botões a matriz
    m_chooser.addOption("MODO 4", DashOpt3);                // Adiciona botões a matriz
    m_chooser.addOption("MODO 5", DashOpt4);                // Adiciona botões a matriz

    SmartDashboard.putData("Escolha o Autonomo", m_chooser);

    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putBoolean("CHECK 1", CheckBox);              // Cria o Botão no DashBoard
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit()
  {
  }

  @Override
  public void teleopPeriodic()
  {
    double Gatilho = Gat1_D.getRawAxis(3);
    Boolean Invert = BTN_1.getAButton();

    m_autoSelected = m_chooser.getSelected();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable smartDashboard = inst.getTable("SmartDashboard");

    boolean DashBTN1 = smartDashboard.getEntry("CHECK 1").getBoolean(false);  // Pega os valores do Botão no SmartDashboard

    //System.out.println("Test="+buttonState);

    switch(m_autoSelected)
    {
      case "0":
        Gatilho = Gatilho/1;
        break;
      case "1":
        Gatilho = Gatilho/2;
        break;
      case "2":
        Gatilho = Gatilho/3;
        break;
      case "3":
        Gatilho = Gatilho/4;
        break;
      case "4":
        Gatilho = Gatilho/5;
        break;
      default:
        break;
    }

    if((Invert == true) | (DashBTN1 == true))
    {
      Gatilho = Gatilho*(-1);
    }

    SmartDashboard.putNumber("Joy1GatD", Gatilho);    // Cria a Indicação do Gatilho Direito
    SmartDashboard.putBoolean("Invert?", Invert);     // Cria o indicador de inversão dos motores

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
