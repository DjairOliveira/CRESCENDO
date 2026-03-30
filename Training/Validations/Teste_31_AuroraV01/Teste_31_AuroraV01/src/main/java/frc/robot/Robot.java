// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Código: Swerve Drive V1.9
 * Equipe: 9168 AGROBOT
 * Desenvolvido por: Djair Oliveira & João Pistore.
 * 
 * As bibliotecas utilizadas e as versões dos softwares são baseadas na versão 2023.
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
  // private Intake Itk;

  private static final String DashDefault  = "0";
  private static final String DashOpt1     = "1";
  private static final String DashOpt2     = "2";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  double VMotor=0;
  double VTransEixo=0;

  double Direcao=0;
  double Translacao=0;
  int Automode=0;
  int[] DriverAnalog_Stimulo = new int[4];
  boolean Var=false;

  @Override
  public void robotInit() 
  {
    swerve = new Swervedrive();
    // Itk = new Intake();

    m_chooser.setDefaultOption("Auto: Inativo", DashDefault);      // Cria o botão principal
    m_chooser.addOption("Auto: Move Front", DashOpt1);             // Adiciona botões a matriz
    m_chooser.addOption("Auto: Move Left", DashOpt2);              // Adiciona botões a matriz
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
      default:
      Automode=(-1);
        break;
    }

    // Modo_AlinhamentoITK=BTN_CTR[3];
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
      VMotor=0;
      Direcao=0;
      Translacao=0;
      Var=false;
    }
    if(Automode==1)
    {
      //VMotor=0.2;
      //Direcao=179.999;
      //Translacao=0;
      //Var=true;
      // swerve.MovAuto
      
      
    }
    // if(Automode==2)
    // {
    //   VMotor=0.5;
    //   Direcao=-90;
    //   Translacao=0;
    //   Var=true;
    // }
  }

  @Override
  public void teleopInit()
  {
    swerve.setAllTMotorsInvert(false);
    swerve.setAllIdleMode(IdleMode.kBrake);
    swerve.setPIDConfigAngulo(0.016,0.3);
    swerve.setPIDConfigTracao(0, 0);
    // Itk.setIncliIdleMode(IdleMode.kCoast);
  }

  @Override
  public void teleopPeriodic() 
  {
    swerve.MovTeleOp();
  }

  @Override
  public void disabledInit() 
  {}

  @Override
  public void disabledPeriodic() 
  {
    swerve.zerador();
    swerve.setAllIdleMode(IdleMode.kCoast);
    swerve.setAllITKMotorsOutPut(0);
    //Itk.setIncliIdleMode(IdleMode.kCoast);
    //Itk.setAllTMotorsOutPut(0);

    VMotor = 0;
    Direcao = 0;
    Translacao = 0;
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
