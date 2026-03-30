// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Código: Swerve Drive V1.9
 * Equipe: 9168 AGROBOT
 * Desenvolvido por: Djair Oliveira & João Pistore.
 * 
 * As bibliotecas utilizadas e as versões dos softwares são baseadas na versão 2024.
 * 
 * O código em questão foi realizado em 2 meses e meio, e contou com o auxílio do Luca Carvalho
 * da equipe 1156 Under Control, no esclarecimento de algumas dúvidas e do chatGPT para o desenvolvimento de algumas funções.
 * 
 * O código ainda possui alguns erros de locomoção, o desenvolvimento do código não foi realizado envolvendo
 * calculos matématicos muito complexos, como por exemplo calculos relacionados a velocidade angular.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Swervedrive;

import com.revrobotics.CANSparkBase.IdleMode;

public class Robot extends TimedRobot {

  private Swervedrive swerve;

  private static final String DashDefault  = "0";
  private static final String DashOpt1     = "1";
  private static final String DashOpt2     = "2";
  private static final String DashOpt3     = "3";
  private static final String DashOpt4     = "4";
  private static final String DashOpt5     = "5";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  int Automode=0;

  @Override
  public void robotInit() 
  {
    swerve = new Swervedrive();

    m_chooser.setDefaultOption("Auto: Inativo", DashDefault);      // Cria o botão principal
    m_chooser.addOption("Auto: Basic Center", DashOpt1);             // Adiciona botões a matriz
    m_chooser.addOption("Auto: Left Center", DashOpt2);              // Adiciona botões a matriz
    m_chooser.addOption("Auto: Right Center", DashOpt3); 
    m_chooser.addOption("Auto: Basic Right", DashOpt4);
    m_chooser.addOption("Auto: Basic Left", DashOpt5);
    SmartDashboard.putData("MODO AUTONOMO SELECT:", m_chooser);
    SmartDashboard.putData(CommandScheduler.getInstance());

  }

  @Override
  public void robotPeriodic() 
  {
    m_autoSelected = m_chooser.getSelected();
    switch(m_autoSelected)
    {
      case "0":
      Automode=0;
        break;
      case "1":
      Automode=1;
        break;
      case "2":
      Automode=2;
        break;
      case "3":
      Automode=3;
        break;
      case "4":
      Automode=4;
        break;
      case "5":
      Automode=5;
        break;
      default:
      Automode=(-1);
        break;
    }

    swerve.printDebugvalues();
    swerve.periodic();
    
  }

  @Override
  public void autonomousInit() 
  {
    swerve.setPIDConfigAngulo(0.016, 0.3);
    swerve.setPIDConfigTracao(1, 0.4);
  }

  @Override
  public void autonomousPeriodic() 
  {
    if(Automode==0)
    {
      // Nada
    }
    if(Automode==1)
    {
      swerve.MovAuto1();
    }
    if(Automode==2)
    {
      swerve.MovAuto2();
    }
    if (Automode == 3){
      swerve.MovAuto3();
    }
    if (Automode == 4){
      swerve.MovAuto4();
    }
    if (Automode == 5){
      swerve.MovAuto5();
    }
  }

  @Override
  public void teleopInit()
  {
    swerve.Disable_Autonomous();
    swerve.setAllTMotorsInvert(false);
    swerve.setAllIdleMode(IdleMode.kBrake);
    swerve.setPIDConfigAngulo(0.016,0.3);
    swerve.setPIDConfigTracao(0, 0);
  }

  @Override
  public void teleopPeriodic() 
  {
    swerve.MovTeleOp();
  }

  @Override
  public void disabledInit() 
  {

  }

  @Override
  public void disabledPeriodic() 
  {
    swerve.zerador();
    swerve.setAllIdleMode(IdleMode.kCoast);
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
