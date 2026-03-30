// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

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
import edu.wpi.first.wpilibj.GenericHID;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANPIDController;

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
  private CANSparkMax DBR;
  private CANSparkMax DBL;
  private CANSparkMax DFR;
  private CANSparkMax DFL;

  private CANSparkMax ABR;
  private CANSparkMax ABL;
  private CANSparkMax AFR;
  private CANSparkMax AFL;

  private CANCoder BR_CANCoder;
  private CANCoder BL_CANCoder;
  private CANCoder FR_CANCoder;
  private CANCoder FL_CANCoder;

  private XboxController Controller_1;

  private static final String DashDefault  = "0";
  private static final String DashOpt1     = "1";
  private static final String DashOpt2     = "2";
  private static final String DashOpt3     = "3";
  private static final String DashOpt4     = "4";

  private String m_autoSelected;
  private boolean CheckBox;

  double JoyLeft1Graus_Anterior=0;
  boolean Caminho=false;

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static boolean InRange(double valor, double valorMinimo, double valorMaximo)
  {
    return valor >= valorMinimo && valor <= valorMaximo;
  }
    /*
    public void teste (String[] args)
    {

    }
    */
  // Função para normalizar um ângulo para o intervalo [0, 360)
  public double normalizarAngulo (double angulo)
  {
    angulo = angulo % 360;
    if (angulo < 0) angulo += 360;
    return angulo;
  }
    // Função para calcular o menor caminho entre dois ângulos
  public double calcularMenorCaminho(double anguloAtual, double novoAngulo)
  {
    // Normalizar os ângulos para o intervalo [0, 360)
    anguloAtual = normalizarAngulo(anguloAtual);
    novoAngulo = normalizarAngulo(novoAngulo);
    // Calcular a diferença entre os ângulos
    double diferenca = novoAngulo - anguloAtual;
    // Verificar se o caminho mais curto é através de 0 ou 360
    if (Math.abs(diferenca) > 180)
    {
      if (diferenca > 0)
      {
        diferenca -= 360;
        Caminho=true;
      }
      else 
      {
        diferenca += 360;
        Caminho=false;
      }
    }
    return diferenca;
  }

  @Override
  public void robotInit()
  {
    DBR = new CANSparkMax(1, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor
    DBL = new CANSparkMax(3, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor
    DFL = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor
    DFR = new CANSparkMax(7, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor

    ABL = new CANSparkMax(4, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor
    AFL = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor
    ABR = new CANSparkMax(2, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor
    AFR = new CANSparkMax(8, CANSparkMax.MotorType.kBrushless);   // Define o CAN ID do SparkMax e qual o tipo do Motor

    BR_CANCoder = new CANCoder(12);
    BL_CANCoder = new CANCoder(9);
    FL_CANCoder = new CANCoder(10);
    FR_CANCoder = new CANCoder(11);

    ABL.setIdleMode(IdleMode.kCoast);
    AFL.setIdleMode(IdleMode.kCoast);
    ABR.setIdleMode(IdleMode.kCoast);
    AFR.setIdleMode(IdleMode.kCoast);

    DBR.setIdleMode(IdleMode.kBrake);
    DBL.setIdleMode(IdleMode.kBrake);
    DFL.setIdleMode(IdleMode.kBrake);
    DFR.setIdleMode(IdleMode.kBrake);
/*
    ABL.getPIDController().setP(2);
    ABL.getPIDController().setI(0);
    ABL.getPIDController().setD(0);


    AFL.getPIDController().setP(1);
    AFL.getPIDController().setI(0);
    AFL.getPIDController().setD(0);
    AFL.getPIDController().setOutputRange(-0.5, 0.5);

    ABR.getPIDController().setP(1);
    ABR.getPIDController().setI(0);
    ABR.getPIDController().setD(0);
    ABR.getPIDController().setOutputRange(-0.5, 0.5);
    

    AFR.getPIDController().setP(1);
    AFR.getPIDController().setI(0);
    AFR.getPIDController().setD(0);
    AFR.getPIDController().setOutputRange(-0.5, 0.5);
    
    ABL.getPIDController().setPositionPIDWrappingEnabled(true);
    ABL.getPIDController().setPositionPIDWrappingMaxInput(359.99999);
    ABL.getPIDController().setPositionPIDWrappingMinInput(0);
*/
    Controller_1 = new XboxController(0);

    m_chooser.setDefaultOption("MODO 1", DashDefault);      // Cria o botão principal
    m_chooser.addOption("MODO 2", DashOpt1);                // Adiciona botões a matriz
    m_chooser.addOption("MODO 3", DashOpt2);                // Adiciona botões a matriz
    m_chooser.addOption("MODO 4", DashOpt3);                // Adiciona botões a matriz
    m_chooser.addOption("MODO 5", DashOpt4);                // Adiciona botões a matriz

    SmartDashboard.putData("Escolha o Autonomo", m_chooser);

    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putBoolean("CHECK 1", CheckBox);           // Cria o Botão no DashBoard
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
    double Gatilho = Controller_1.getRawAxis(3);
    Boolean Invert = Controller_1.getAButton();
    boolean Status=false;
    boolean CTR = false;
    double Speed = 0;

    double BL_CCD = BL_CANCoder.getAbsolutePosition();        // Back Left CanCoder
    //double BL_ENEO = ABL.getEncoder().getPosition();        // Back Left Encoder Inteno NEO
    //double BL_ENEO_ABS = Math.abs(BL_ENEO/0.4111586111111111);
    m_autoSelected = m_chooser.getSelected();

    double JoyLeft1Rad = Math.atan2(Controller_1.getRawAxis(0), Controller_1.getRawAxis(1));
    double JoyLeft1Graus = Math.toDegrees(JoyLeft1Rad);
    JoyLeft1Graus = (JoyLeft1Graus < 0) ? JoyLeft1Graus + 360 : JoyLeft1Graus;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable smartDashboard = inst.getTable("SmartDashboard");

    boolean DashBTN1 = smartDashboard.getEntry("CHECK 1").getBoolean(false);  // Pega os valores do Botão no SmartDashboard
    
    switch(m_autoSelected)
    {
      case "0":
        Gatilho /= 1;
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
      //ABL.getEncoder().setPosition(0);
      //BL_ENEO=0;
    }

    if(((((Controller_1.getRawAxis(0)>=0.3) || (Controller_1.getRawAxis(0)<=-0.3))) || (((Controller_1.getRawAxis(1)>=0.3) || (Controller_1.getRawAxis(1)<=-0.3)))))
    {
      Status=true;
      if((JoyLeft1Graus>(BL_CCD+1)) && (JoyLeft1Graus<(BL_CCD-1)))
      {
        Speed=0;
        CTR=true;
      }
      else CTR=false;

      if(CTR==false)
      {
        //System.out.println("Menor caminho: " + menorCaminho + " graus");
        if(BL_CCD>=JoyLeft1Graus) Speed=0.5;
        if(BL_CCD<=JoyLeft1Graus) Speed=-0.5;

        if(InRange(BL_CCD,(JoyLeft1Graus-50),(JoyLeft1Graus+50))==true)
        {
          if(BL_CCD>=JoyLeft1Graus) Speed=0.2;
          if(BL_CCD<=JoyLeft1Graus) Speed=-0.2;
        }
        if(InRange(BL_CCD,(JoyLeft1Graus-10),(JoyLeft1Graus+10))==true)
        {
          if(BL_CCD>=JoyLeft1Graus) Speed=0.05;
          if(BL_CCD<=JoyLeft1Graus) Speed=-0.05;
        }
        if(InRange(BL_CCD,(JoyLeft1Graus-3),(JoyLeft1Graus+3))==true)
        {
          if(BL_CCD>=JoyLeft1Graus) Speed=0.02;
          if(BL_CCD<=JoyLeft1Graus) Speed=-0.02;
        }
        if(InRange(BL_CCD,(JoyLeft1Graus-1),(JoyLeft1Graus+1))==true)
        {
          Speed=0;
        }
      }
      else Speed = 0;
      
      System.out.println("Valor que ta saindo: "+Speed);
    }
    else 
    {
      Speed=0;
      Status=false;
    }
  
    // double anguloAtual = 2;
    // double novoAngulo = 340;
    // double menorCaminho = calcularMenorCaminho(BL_CCD, JoyLeft1Graus);
    // System.out.println("Menor caminho: " + menorCaminho + " graus");
    /*
    if(BL_ENEO_ABS>359.9999)
    {
      ABL.getEncoder().setPosition(0);
      BL_ENEO_ABS=0;
    }
    */

    /*
    if(((JoyLeft1Graus-1)<=JoyLeft1Graus_Anterior) & ((JoyLeft1Graus+1)>=JoyLeft1Graus_Anterior) | (((Controller_1.getRawAxis(0)<=0.2) & (Controller_1.getRawAxis(0)>=-0.2)) & ((Controller_1.getRawAxis(1)<=0.2) & (Controller_1.getRawAxis(1)>=-0.2))))
    {
      ABL.getPIDController().setP(0);
      Status=true;
    }
    else
     {
      ABL.getPIDController().setP(2);
      ABL.getPIDController().setOutputRange(-0.5, 0.5);
      Status=false;
    }
    
    ABL.getPIDController().setReference(JoyLeft1Graus, ControlType.kPosition);
    */
    
    //JoyLeft1Graus_Anterior = JoyLeft1Graus;

    SmartDashboard.putNumber("PIDSENDMOTOR", ABL.getAppliedOutput());
    SmartDashboard.putNumber("BLABSPosition", BL_CCD);
    //SmartDashboard.putNumber("BL_EncoderABS", BL_ENEO_ABS);
    //SmartDashboard.putNumber("BL_Encoder", BL_ENEO);

    SmartDashboard.putNumber("Joy", JoyLeft1Graus);
    SmartDashboard.putBoolean("Invert?", Invert);     // Cria o indicador de inversão dos motores
    SmartDashboard.putBoolean("Status", Status);     // Cria o indicador de inversão dos motores
    SmartDashboard.putBoolean("CTR", CTR);     // Cria o indicador de inversão dos motores
    SmartDashboard.putBoolean("Caminho", Caminho);     // Cria o indicador de inversão dos motores
    
    if(calcularMenorCaminho(BL_CCD, JoyLeft1Graus) < 0)
    {
      Speed *= -1;
    }

    DBL.set(Gatilho);
    DBR.set(Gatilho);
    DFL.set(Gatilho);
    DFR.set(Gatilho);

    ABL.set(Speed);
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
