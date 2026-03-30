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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.CANcoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private CANSparkMax TMSE;                         // TRAÇÃO MOTOR SUPERIOR ESQUERDO
  private CANSparkMax TMSD;                         // TRAÇÃO MOTOR SUPERIOR DIREITO
  private CANSparkMax TMIE;                         // TRAÇÃO MOTOR INFERIOR ESQUERDO
  private CANSparkMax TMID;                         // TRAÇÃO MOTOR INFERIOR DIREITO

  private CANSparkMax AMSE;                         // ANGULO MOTOR SUPERIOR ESQUERDO
  private CANSparkMax AMSD;                         // ANGULO MOTOR SUPERIOR DIREITO
  private CANSparkMax AMIE;                         // ANGULO MOTOR INFERIOR ESQUERDO
  private CANSparkMax AMID;                         // ANGULO MOTOR INFERIOR DIREITO

  private CANSparkMax INC;                         // MOTOR DE INCLINAÇÃO DIREITA

  private CANSparkMax ITKP;                         // INTAKE PRINCIPAL
  private CANSparkMax ITKDS;                         // INTAKE DISPARO SUPERIOR
  private CANSparkMax ITKDI;                         // INTAKE DISPARO INFERIOR

  private CANcoder CANC_SE;
  private CANcoder CANC_SD;
  private CANcoder CANC_IE;
  private CANcoder CANC_ID;

  DigitalInput FCINC;

  private static XboxController controller;

  private Pigeon2 Pigeon;

  private NetworkTable LimeTable;
  private int PipelineSelect = 1;
  private int LEDMODE=1;

  private static final String DashDefault  = "0";
  private static final String DashOpt1     = "1";
  private static final String DashOpt2     = "2";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private static final String Pipe0  = "0";
  private static final String Pipe1  = "1";
  private static final String Pipe2  = "2";
  private String m_PipeSelect;
  private final SendableChooser<String> m_Pipe = new SendableChooser<>();

  private static final String LimeLED0  = "0";
  private static final String LimeLED1  = "1";
  private static final String LimeLED2  = "2";
  private static final String LimeLED3  = "3";
  
  private String m_LimeLEDSelect;
  private final SendableChooser<String> m_LimeLED = new SendableChooser<>();
  double VMotor=0;
  double VTransEixo=0;
  double Direcao=0;
  double Translacao=0;
  int Automode=0;
  boolean[] Analog_Stimu = new boolean[2];
  boolean[] BTN_CTR = new boolean[5];

  boolean Coletar_Low=false;
  boolean Deposita_Amp=false;
  boolean Warning_Benga=false;
  
  private double Ref_Encoder_SD = 16.417020784085121;     // Referência do Encoder: 360°/21.38 = 16.83816651075772 (21.38 = 360° do Encoder)  21,928461
  private double Ref_Encoder_SE = 16.744341807830770;      // 21,4998
  private double Ref_Encoder_IE = 16.762829431505402;     // 21,476088
  private double Ref_Encoder_ID = 16.417020784085121;     //21,928461

  private double Ref_PID_P = 0.016;                     // Referência Proporcional PID: Valor de SetPoint da Proporcional PID
  private double Ref_Min_Max_Wrapping = 0.3;           // Range Máximo e Minimo PID Wrapping: Valor minimo e máximo que vai sair para o motor

  private boolean[] Var_Ctr = new boolean[7];         // Variaveis de controle

  boolean Tg=false, Ctr=false, Coletou=false, Coletar=false;

  int cont1=0, cont2=0, cont3=0, Mult=0;
  double VeloDisp=0, VeloColeta=0;

  public static boolean Analog_Stimulated(int Axisx, int Axisy, double Range)   // Verifica se o Joystick saiu da posição inicial
  {
    return (controller.getRawAxis(Axisx) <= (-Range) || controller.getRawAxis(Axisx) >= Range || controller.getRawAxis(Axisy) <= (-Range) || controller.getRawAxis(Axisy) >= Range);
  }

  public static void SPARKMAX_CONFIG_TRACAO(CANSparkMax Motor)                        // Configura o PID e range de saida para o motor Alvo
  {
    Motor.getPIDController().setP(1);
    Motor.getPIDController().setI(0);
    Motor.getPIDController().setD(0);
    Motor.getPIDController().setOutputRange(-1, 1);
  }

  public static void SPARKMAX_CONFIG_ANGULO (CANSparkMax Motor, double Ref_Enc, double Min_Max_Wpg)   // Configura o PID, o range de saida e o Modo Wrapping
  {
    Motor.getPIDController().setP(0.016);
    Motor.getPIDController().setI(0);
    Motor.getPIDController().setD(0);
    Motor.getPIDController().setOutputRange(Min_Max_Wpg*(-1), Min_Max_Wpg);
    Motor.getPIDController().setPositionPIDWrappingEnabled(true);
    Motor.getPIDController().setPositionPIDWrappingMaxInput(360);
    Motor.getPIDController().setPositionPIDWrappingMinInput(0);
    Motor.getEncoder().setPositionConversionFactor(Ref_Enc);
  }

  public static void SET_ALL_PID_P(CANSparkMax Inf_Esq, CANSparkMax Inf_Dir, CANSparkMax Sup_Esq, CANSparkMax Sup_Dir, double PID_P)  // Configura o P de todos os motores
  {
    Inf_Esq.getPIDController().setP(PID_P);
    Inf_Dir.getPIDController().setP(PID_P);
    Sup_Esq.getPIDController().setP(PID_P);
    Sup_Dir.getPIDController().setP(PID_P);
  }

  public static double ALINHAMENTO(CANSparkMax Motor1, CANSparkMax Motor2, CANSparkMax Motor3, CANSparkMax Motor4, double ABS_Alvo, double Angle_Setpoint)
  {
    double v_Motor=0;

    if(ABS_Alvo>(Angle_Setpoint+20)) v_Motor=-0.2;
    if(ABS_Alvo<(Angle_Setpoint-20)) v_Motor=0.2;
    if(ABS_Alvo>(Angle_Setpoint+1) && ABS_Alvo<=(Angle_Setpoint+20)) v_Motor = -0.035;
    if(ABS_Alvo>=(Angle_Setpoint-21) && ABS_Alvo<=(Angle_Setpoint-1)) v_Motor = 0.035;
    if(ABS_Alvo>(Angle_Setpoint-1) && ABS_Alvo<=(Angle_Setpoint+1))
    {
      v_Motor = 0;

      Motor1.getEncoder().setPosition(0);
      Motor2.getEncoder().setPosition(0);
      Motor3.getEncoder().setPosition(0);
      Motor4.getEncoder().setPosition(0);
    }
    return v_Motor;
  }
  public double getPositionRef(CANSparkMax Motor, double PoseDesejada, double PoseAtual) {
    // esta invertendo varias vezes fazendo ele querer ir para a 
    // posição real e a invertida ao mesmo tempo e gerando tec!! tec!! tec!!
    // isso aqui ta esquisitão to fazendo uns bagulho meio duvidoso eu acho
    // sei la :)
    double pose4joy = PoseAtual + 180;
    pose4joy = (pose4joy >= 360) ? pose4joy - 360: pose4joy; 
    pose4joy -= 180;

    double Ap = pose4joy + 90;
    double An = pose4joy - 90;
    boolean Invert = !Motor.getInverted();

    Ap = (Ap > 180) ? Ap -= 360 : Ap;
    An = (An < (-180)) ? An += 360 : An;

    if (PoseAtual > 90 && PoseAtual < 270) {
      if (PoseDesejada > An && PoseDesejada < Ap) {
        PoseDesejada += 180;
        PoseDesejada = (PoseDesejada >= 180) ? PoseDesejada -= 360 : PoseDesejada;
        Motor.setInverted(Invert);
      }
    }
    else {
      if (PoseDesejada < An || PoseDesejada > Ap) {
        PoseDesejada += 180;
        PoseDesejada = (PoseDesejada >= 180) ? PoseDesejada -= 360 : PoseDesejada;
        Motor.setInverted(Invert);
      }
    }
    return PoseDesejada;
  }
  public double getPosABS(Boolean getmotorinverted, double PosicaoABS) {
    if (getmotorinverted) {
      PosicaoABS += 180;
      PosicaoABS = (PosicaoABS >= 360) ? PosicaoABS -= 360 : PosicaoABS;
    }
    return PosicaoABS;
  }


  @Override
  public void robotInit() 
  {
    controller = new XboxController(0);

    FCINC = new DigitalInput(0);

    Pigeon = new Pigeon2(13);
    
    TMIE = new CANSparkMax(7,MotorType.kBrushless);
    AMIE = new CANSparkMax(8,MotorType.kBrushless);
    TMIE.setInverted(false);
    AMIE.setInverted(true);
    CANC_IE = new CANcoder(18);

    TMID = new CANSparkMax(5,MotorType.kBrushless);
    AMID = new CANSparkMax(6,MotorType.kBrushless);
    TMID.setInverted(false);
    AMID.setInverted(true);
    CANC_ID = new CANcoder(17);

    TMSE = new CANSparkMax(1,MotorType.kBrushless);
    AMSE = new CANSparkMax(2,MotorType.kBrushless);
    TMSE.setInverted(false);
    AMSE.setInverted(true);
    CANC_SE = new CANcoder(15);

    TMSD = new CANSparkMax(3,MotorType.kBrushless);
    AMSD = new CANSparkMax(4,MotorType.kBrushless);
    TMSD.setInverted(false);
    AMSD.setInverted(true);
    CANC_SD = new CANcoder(16);

    INC = new CANSparkMax(9,MotorType.kBrushless);
   
    ITKP = new CANSparkMax(12,MotorType.kBrushless);
    ITKDS = new CANSparkMax(10,MotorType.kBrushless);
    ITKDI = new CANSparkMax(11,MotorType.kBrushless);


    LimeTable = NetworkTableInstance.getDefault().getTable("limelight");

    
    // INCD.getPIDController().setP(0.2);
    // INCD.getPIDController().setI(0);
    // INCD.getPIDController().setD(0);
    // INCD.getPIDController().setOutputRange(-0.8, 0.8);

    // INCE.getPIDController().setP(0.2);
    // INCE.getPIDController().setI(0);
    // INCE.getPIDController().setD(0);
    // INCE.getPIDController().setOutputRange(-0.8, 0.8);

    SPARKMAX_CONFIG_TRACAO(TMIE);
    SPARKMAX_CONFIG_TRACAO(TMID);
    SPARKMAX_CONFIG_TRACAO(TMSD);
    SPARKMAX_CONFIG_TRACAO(TMSE);

    SPARKMAX_CONFIG_ANGULO(AMIE, Ref_Encoder_IE, Ref_Min_Max_Wrapping);
    SPARKMAX_CONFIG_ANGULO(AMID, Ref_Encoder_ID, Ref_Min_Max_Wrapping);
    SPARKMAX_CONFIG_ANGULO(AMSD, Ref_Encoder_SD, Ref_Min_Max_Wrapping);
    SPARKMAX_CONFIG_ANGULO(AMSE, Ref_Encoder_SE, Ref_Min_Max_Wrapping);

    TMIE.getEncoder().setPosition(0);
    AMIE.getEncoder().setPosition(0);

    TMID.getEncoder().setPosition(0);
    AMID.getEncoder().setPosition(0);

    TMSE.getEncoder().setPosition(0);
    AMSE.getEncoder().setPosition(0);

    TMSD.getEncoder().setPosition(0);
    AMSD.getEncoder().setPosition(0);

    INC.getEncoder().setPosition(0);

    double ABS_CSD = ((CANC_SD.getAbsolutePosition().getValueAsDouble()*360) % 360);
    double ABS_CID = ((CANC_ID.getAbsolutePosition().getValueAsDouble()*360) % 360);
    double ABS_CSE = ((CANC_SE.getAbsolutePosition().getValueAsDouble()*360) % 360);
    double ABS_CIE = ((CANC_IE.getAbsolutePosition().getValueAsDouble()*360) % 360);

    if (ABS_CSD<0) ABS_CSD += 360;
    if (ABS_CID<0) ABS_CID += 360;
    if (ABS_CSE<0) ABS_CSE += 360;
    if (ABS_CIE<0) ABS_CIE += 360;

    AMIE.getEncoder().setPosition(ABS_CIE);
    AMID.getEncoder().setPosition(ABS_CID);
    AMSE.getEncoder().setPosition(ABS_CSE);
    AMSD.getEncoder().setPosition(ABS_CSD);

    Pigeon.setYaw(0);

    m_chooser.setDefaultOption("Auto: Inativo", DashDefault);      // Cria o botão principal
    m_chooser.addOption("Auto: Move Front", DashOpt1);                // Adiciona botões a matriz
    m_chooser.addOption("Auto: Move Left", DashOpt2);                // Adiciona botões a matriz
    SmartDashboard.putData("MODO AUTONOMO SELECT:", m_chooser);
    SmartDashboard.putData(CommandScheduler.getInstance());

    m_Pipe.setDefaultOption("PIPE: Nota", Pipe0);
    m_Pipe.addOption("PIPE: Tags", Pipe1);
    m_Pipe.addOption("PIPE: Indefinida", Pipe2);
    SmartDashboard.putData("PIPELINE SELECT:", m_Pipe);
    SmartDashboard.putData(CommandScheduler.getInstance());

    m_LimeLED.setDefaultOption("MODO: Default", LimeLED0);
    m_LimeLED.addOption("MODO: Off", LimeLED1);
    m_LimeLED.addOption("MODO: On", LimeLED2);
    m_LimeLED.addOption("MODO: On Process", LimeLED3);
    SmartDashboard.putData("LIMELED MODE:", m_LimeLED);
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  @Override
  public void robotPeriodic() 
  {
    double MTSD=0;    // Motor Tração Superior Direito
    double MTID=0;    // Motor Tração Inferior Direito
    double MTSE=0;    // Motor Tração Superior Esquerdo
    double MTIE=0;    // Motor Tração Inferior Esquerdo

    double MAIE=0;   // Motor Angulo Inferior Esquerdo
    double MASE=0;   // Motor Angulo Superior Esquerdo
    double MAID=0;   // Motor Angulo Inferior Direito
    double MASD=0;   // Motor Angulo Superior Direito

    double PosDIE = 0;  // Variavel que esta assumindo
    double PosDSE = 0;  // o valor da posição desejada
    double PosDID = 0;  // que sera a posição do Joy
    double PosDSD = 0;  // ou o inverso dela             // as funções dela estão comentadas

    double Velocidade = VMotor;
    double JoyLeft_degrees = Direcao;
    double JoyRight_degrees = Translacao;

    double V_Inc = 0;

    double Yaw    = Pigeon.getYaw().getValueAsDouble();
    double Pitch  = Pigeon.getPitch().getValueAsDouble();
    double Roll   = Pigeon.getRoll().getValueAsDouble();

    Yaw*=(-1);
    double absoluteYaw = Yaw % 360;

    double ABS_CSD = (((CANC_SD.getAbsolutePosition().getValueAsDouble()*360) % 360) - 82.3);
    double ABS_CID = (((CANC_ID.getAbsolutePosition().getValueAsDouble()*360) % 360) - 37.2);
    double ABS_CSE = (((CANC_SE.getAbsolutePosition().getValueAsDouble()*360) % 360) - 156.6);
    double ABS_CIE = (((CANC_IE.getAbsolutePosition().getValueAsDouble()*360) % 360) - 51.5);


    if (ABS_CSD<0) ABS_CSD += 360;
    if (ABS_CID<0) ABS_CID += 360;
    if (ABS_CSE<0) ABS_CSE += 360;
    if (ABS_CIE<0) ABS_CIE += 360;

    /*
    AMIE.getEncoder().setPosition(getPosABS(TMIE.getInverted(), ABS_CIE)); // seta para a posiçao do cancoder
    AMID.getEncoder().setPosition(getPosABS(TMID.getInverted(), ABS_CID)); // ou para o oposto dela caso
    AMSE.getEncoder().setPosition(getPosABS(TMSE.getInverted(), ABS_CSE)); // o motor da tração esteja
    AMSD.getEncoder().setPosition(getPosABS(TMSD.getInverted(), ABS_CSD)); // inverto
    */

    AMIE.getEncoder().setPosition(ABS_CIE);                  // modo convencional
    AMID.getEncoder().setPosition(ABS_CID);                  // somente seta os encoderes dos
    AMSE.getEncoder().setPosition(ABS_CSE);                  // motores para a posição do 
    AMSD.getEncoder().setPosition(ABS_CSD);                  // cancoder

    double TargetX = LimeTable.getEntry("tx").getDouble(0.0);
    double TargetY = LimeTable.getEntry("ty").getDouble(0.0);
    double TargetArea = LimeTable.getEntry("ta").getDouble(0.0);

    double ITKP_Current = ITKP.getOutputCurrent();

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

    m_PipeSelect = m_Pipe.getSelected();
    switch(m_PipeSelect)
    {
      case "0":
        PipelineSelect=0;
        break;
      case "1":
        PipelineSelect=1;
        break;
      case "2":
        PipelineSelect=2;
        break;
      default:
        PipelineSelect=1;
        break;
    }

    m_LimeLEDSelect = m_LimeLED.getSelected();
    switch(m_LimeLEDSelect)
    {
      case "0":
        LEDMODE=0;
        break;
      case "1":
        LEDMODE=1;
        break;
      case "2":
        LEDMODE=2;
        break;
      case "3":
        LEDMODE=3;
        break;
      default:
        LEDMODE=1;
        break;
    }

  if (absoluteYaw<0) absoluteYaw += 360;          // Lógica de retorno ao cruzar os 360° ou decrementar de 0°
  if ((Analog_Stimu[0]==true && Analog_Stimu[1]==false) || Var_Ctr[3]==true) 
  {
    if(Var_Ctr[0]==false)
    {
      AMID.getPIDController().setP(Ref_PID_P);
      AMIE.getPIDController().setP(Ref_PID_P);
      AMSD.getPIDController().setP(Ref_PID_P);
      AMSE.getPIDController().setP(Ref_PID_P);
      AMID.getPIDController().setReference(315, ControlType.kPosition);
      AMIE.getPIDController().setReference(225, ControlType.kPosition);
      AMSD.getPIDController().setReference(45, ControlType.kPosition);
      AMSE.getPIDController().setReference(135, ControlType.kPosition);
      TMIE.setInverted(false);
      TMID.setInverted(false);
      TMSE.setInverted(false);
      TMSD.setInverted(false);
      Var_Ctr[0]=true;
    }

    Velocidade = (VTransEixo/4);
    MTSD = Velocidade;
    MTID = Velocidade;
    MTIE = Velocidade;
    MTSE = Velocidade;
  }
  else
  {
    Var_Ctr[0]=false;
  }
  if(Var_Ctr[0]==false)
  {
    if (Analog_Stimu[1] || Var_Ctr[2]==true)
    {
      if(Var_Ctr[1]==false)
      {
        SET_ALL_PID_P(AMIE, AMID, AMSE, AMSD, Ref_PID_P);
        Var_Ctr[1]=true;
      }

      PosDIE = getPositionRef(TMIE, JoyLeft_degrees, AMIE.getEncoder().getPosition()); //
      PosDID = getPositionRef(TMID, JoyLeft_degrees, AMID.getEncoder().getPosition());
      PosDSE = getPositionRef(TMSE, JoyLeft_degrees, AMSE.getEncoder().getPosition());
      PosDSD = getPositionRef(TMSD, JoyLeft_degrees, AMSD.getEncoder().getPosition());


      MAIE = (absoluteYaw + JoyLeft_degrees); // Mandar o JoyLeft_degrees no lugar dos posD** para modo convencional
      MAID = (absoluteYaw + JoyLeft_degrees); // certifique-se de que os encoderes estão sendo igualados
      MASE = (absoluteYaw + JoyLeft_degrees); // corretamente à posição dos cancoderes para o funcionamento
      MASD = (absoluteYaw + JoyLeft_degrees); // adequado do modo convencional
      
      /*if(VTransEixo >= 0.15 || VTransEixo <= -0.15)
      {
        if((JoyRight_degrees>=135 || JoyRight_degrees<=-134.999) && (JoyLeft_degrees>=135 || JoyLeft_degrees<=-134.999))
        {
          if(absoluteYaw>=315 || absoluteYaw<=44.999)
          {
            MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
            MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
            MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
            MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
          }
      
          if(absoluteYaw>=45 && absoluteYaw<=134.999)
          {
            MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
            MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
            MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
            MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
          }

          if(absoluteYaw>=135 && absoluteYaw<=224.999)
          {
            MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
            MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
            MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
            MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
          }
          if(absoluteYaw>=225 && absoluteYaw<=314.999)
          {
            MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
            MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
            MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
            MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
          }
        }
        if((JoyRight_degrees>=(-135) && JoyRight_degrees<=(-44.999)) && (JoyLeft_degrees>=(-135) && JoyLeft_degrees<=(-44.999)))
        {
          if(absoluteYaw>=315 || absoluteYaw<=44.999)
          {
            MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
            MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
            MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
            MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
          }
      
          if(absoluteYaw>=45 && absoluteYaw<=134.999)
          {
            MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
            MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
            MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
            MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
          }

          if(absoluteYaw>=135 && absoluteYaw<=224.999)
          {
            MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
            MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
            MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
            MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
          }
          if(absoluteYaw>=225 && absoluteYaw<=314.999)
          {
            MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
            MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
            MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
            MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
          }
        }
        if((JoyRight_degrees>=45 && JoyRight_degrees<=134.999) && (JoyLeft_degrees>=45 && JoyLeft_degrees<=134.999))
        {
          if(absoluteYaw>=315 || absoluteYaw<=44.999)
          {
            MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
            MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
            MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
            MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
          }
      
          if(absoluteYaw>=45 && absoluteYaw<=134.999)
          {
            MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
            MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
            MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
            MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
          }

          if(absoluteYaw>=135 && absoluteYaw<=224.999)
          {
            MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
            MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
            MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
            MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
          }
          if(absoluteYaw>=225 && absoluteYaw<=314.999)
          {
            MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
            MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
            MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
            MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
          }
        }
        if((JoyRight_degrees>=(-45) && JoyRight_degrees<=44.999) && (JoyLeft_degrees>=(-45) && JoyLeft_degrees<=44.999))
        {
          if(absoluteYaw>=315 || absoluteYaw<=44.999)
          {
            MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
            MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_ID;
            MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
            MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
          }
      
          if(absoluteYaw>=45 && absoluteYaw<=134.999)
          {
            MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
            MAID = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
            MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
            MASE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SE;
          }

          if(absoluteYaw>=135 && absoluteYaw<=224.999)
          {
            MASD = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SD;
            MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
            MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
            MAIE = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_IE;
          }
          if(absoluteYaw>=225 && absoluteYaw<=314.999)
          {
            MASE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_SE;
            MAIE = (absoluteYaw+JoyRight_degrees) / Ref_Encoder_IE;
            MASD = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_SD;
            MAID = (absoluteYaw+JoyLeft_degrees) / Ref_Encoder_ID;
          }
        }
      }
      */

      AMIE.getPIDController().setReference(MAIE ,ControlType.kPosition);
      AMID.getPIDController().setReference(MAID ,ControlType.kPosition);
      AMSE.getPIDController().setReference(MASE ,ControlType.kPosition);
      AMSD.getPIDController().setReference(MASD ,ControlType.kPosition);

      MTSD = Velocidade;
      MTID = Velocidade;
      MTIE = Velocidade;
      MTSE = Velocidade;
    }
    else
    {
      if(BTN_CTR[1] == false)  
      {
        SET_ALL_PID_P(AMIE, AMID, AMSE, AMSD, 0);
      }
      Var_Ctr[1]=false;
    }
  }

    if(BTN_CTR[1]==true)
    {
      SET_ALL_PID_P(AMIE, AMID, AMSE, AMSD, Ref_PID_P);

      AMIE.getPIDController().setReference(0, ControlType.kPosition);
      AMID.getPIDController().setReference(0, ControlType.kPosition);
      AMSE.getPIDController().setReference(0, ControlType.kPosition);
      AMSD.getPIDController().setReference(0, ControlType.kPosition);
      TMIE.setInverted(false);
      TMID.setInverted(false);
      TMSE.setInverted(false);
      TMSD.setInverted(false);
    }

/********************* */

    if(BTN_CTR[2]==true && Var_Ctr[5]==false)
    {
      Deposita_Amp=!Deposita_Amp;
      Var_Ctr[5]=true;
    }
    if(BTN_CTR[2]==false && Var_Ctr[5]==true)
    {
      Var_Ctr[5]=false;
    }

    if(Deposita_Amp==true && Warning_Benga==true)
    {
      V_Inc=1;
    }
    if(Deposita_Amp==false && V_Inc==1)
    {
      V_Inc=0;
    }

    /* ////////////////////////////////// */
    if(BTN_CTR[0]==true && Ctr==false)
    {
      Coletar_Low=!Coletar_Low;
      Ctr=true;
    }
    if(BTN_CTR[0]==false && Ctr==true)
    {
      Ctr=false;
    }

    if(Coletar_Low==true)
    {
      if(Coletou==false)
      {
        V_Inc=-1;                     // Abaixa o ITK até o FIM de curso
        VeloColeta=0.4;
        cont2=0;
        Coletar=true;
        if(cont1<=60) cont1++;        // Timer para remover o pico de corrente incial
      }
      if(cont1>60 && ITKP_Current>=13)
      {
        Var_Ctr[6]=true;
      }
    }
    if(Var_Ctr[6]==true)
    {
      if(cont3<=10) cont3++;
      if(cont3>10)
      {
        controller.setRumble(RumbleType.kBothRumble, 0.5);
        VeloColeta=0;
        Coletar=true;
        Coletou=true;
        Var_Ctr[6]=false;
      }
    }
/* 
    if(Coletar_Low==false && Coletou==false)      // Caso não pegue
    {
      cont1=0;
      cont2=0;
      VeloDisp=0;
      VeloColeta=0;
      Coletou=false;
    }
*/
  // if(Coletou == true)
  // {

  // }
    // if(Coletar_Low==false && Coletou==true)      // Caso pegue
    // {
    //   cont1=0;
    //   cont2=0;
    //   if(INC.getEncoder().getPosition() >= 400) VeloDisp=0.15;
    //   if(INC.getEncoder().getPosition() < 460) VeloDisp=0.4;
    //   Coletou=false;
    // }

    if(Coletar_Low==false  && Coletou==true)
    {
      cont1=0;
      cont2=0;
      cont3=0;
      if(INC.getEncoder().getPosition() >= 400) VeloDisp=0.15;
      if(INC.getEncoder().getPosition() < 460) VeloDisp=1;
      Coletou=false; 
    }
    if((cont2<=50 && VeloDisp==1) || (cont2<=50 && VeloDisp==0.15))
    {
      cont2++;
      if(cont2>30)
      {
       VeloColeta=1;
       Deposita_Amp=false;
      }
    }
    if(cont2>50)
    {
      controller.setRumble(RumbleType.kBothRumble, 0);
      VeloColeta=0;
      VeloDisp=0;
    }

    /* ////////////////////////////////// */

    if(controller.getPOV()==0 && Warning_Benga==true)
    {
      V_Inc=1;
    }
    if(controller.getPOV()==180)
    {
      V_Inc=-1;
    }

    if(INC.getEncoder().getPosition() >= 450)
    {
      V_Inc/=2;
    }

    if(INC.getEncoder().getPosition() >= 480 && V_Inc > 0)      // Proteção de avanço do Benga max 492
    {
      V_Inc=0;
    }

    if(FCINC.get()==true && V_Inc <= 0)      // Proteção de Recuo do Benga
    {
      Warning_Benga=true;
      INC.getEncoder().setPosition(0);
      V_Inc=0;
    }
    

    SmartDashboard.putNumber("PIGEON Yaw", Yaw);
    SmartDashboard.putNumber("PIGEON Pitch", Pitch);
    SmartDashboard.putNumber("PIGEON Roll", Roll);
    SmartDashboard.putNumber("YAW ABSOLUTO", absoluteYaw);
    SmartDashboard.putNumber("Joystick Left", JoyLeft_degrees);
    SmartDashboard.putNumber("Joystick Right", JoyRight_degrees);

    SmartDashboard.putNumber("ENCID", 10.4/Ref_Encoder_ID);
    SmartDashboard.putNumber("ENCIE", 160.6/Ref_Encoder_IE);
    SmartDashboard.putNumber("ENCSE", 202.675/Ref_Encoder_SE);
    SmartDashboard.putNumber("ENCSD", 168.75/Ref_Encoder_SD);

    SmartDashboard.putNumber("ENCODER IE", AMIE.getEncoder().getPosition());
    SmartDashboard.putNumber("ENCODER ID", AMID.getEncoder().getPosition());
    SmartDashboard.putNumber("ENCODER SD", AMSD.getEncoder().getPosition());
    SmartDashboard.putNumber("ENCODER SE", AMSE.getEncoder().getPosition());

    SmartDashboard.putNumber("ABS - CANCODER ID", ABS_CID);
    SmartDashboard.putNumber("ABS - CANCODER SD", ABS_CSD);
    SmartDashboard.putNumber("ABS - CANCODER IE", ABS_CIE);
    SmartDashboard.putNumber("ABS - CANCODER SE", ABS_CSE);

    SmartDashboard.putNumber("CANCODER SD", CANC_SD.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("CANCODER ID", CANC_ID.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("CANCODER IE", CANC_IE.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("CANCODER SE", CANC_SE.getAbsolutePosition().getValueAsDouble());

    SmartDashboard.putBoolean("TMID", TMID.getInverted());
    SmartDashboard.putBoolean("TMSD", TMSD.getInverted());
    SmartDashboard.putBoolean("TMIE", TMIE.getInverted());
    SmartDashboard.putBoolean("TMSE", TMSE.getInverted());

    SmartDashboard.putNumber("MAIE", MAIE);
    SmartDashboard.putNumber("MAID", MAID);
    SmartDashboard.putNumber("MASE", MASE);
    SmartDashboard.putNumber("MASD", MASD);

    
    SmartDashboard.putNumber("TARGET TX", TargetX);
    SmartDashboard.putNumber("TARGET TY", TargetY);
    SmartDashboard.putNumber("TARGET TA", TargetArea);

    SmartDashboard.putNumber("joy Left", JoyLeft_degrees);
    
    LimeTable.getEntry("pipeline").setNumber(PipelineSelect);  // Define o valor da pipeline desejada
    LimeTable.getEntry("ledMode").setNumber(LEDMODE);          // Modo do Led definido para 1

    SmartDashboard.putNumber("ENCODER INC", INC.getEncoder().getPosition());
    
    SmartDashboard.putNumber("POV", controller.getPOV());
    SmartDashboard.putBoolean("Fim de Curso", FCINC.get());

    TMSD.set(MTSD);
    TMID.set(MTID);
    TMIE.set(MTIE);
    TMSE.set(MTSE);

    ITKP.set(VeloColeta);
    ITKDS.set(VeloDisp);
    ITKDI.set(VeloDisp);

    INC.set(V_Inc);
  }

  @Override
  public void autonomousInit() 
  {
    Var_Ctr[0]=false;
    Var_Ctr[1]=false;
    Var_Ctr[2]=false;
    Var_Ctr[3]=false;

    AMIE.setIdleMode(IdleMode.kBrake);
    TMIE.setIdleMode(IdleMode.kBrake);
    AMSE.setIdleMode(IdleMode.kBrake);
    TMSE.setIdleMode(IdleMode.kBrake);
    AMSD.setIdleMode(IdleMode.kBrake);
    TMSD.setIdleMode(IdleMode.kBrake);
    AMID.setIdleMode(IdleMode.kBrake);
    TMID.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void autonomousPeriodic() 
  {
    if(Automode==0)
    {
      VMotor=0;
      Direcao=0;
      Translacao=0;
      Var_Ctr[2]=false;
    }
    if(Automode==1)
    {
      VMotor=0.2;
      Direcao=179.999;
      Translacao=0;
      Var_Ctr[2]=true;
    }
    if(Automode==2)
    {
      VMotor=0.5;
      Direcao=-90;
      Translacao=0;
      Var_Ctr[2]=true;
    }
  }

  @Override
  public void teleopInit() 
  {
    Var_Ctr[0]=false;
    Var_Ctr[1]=false;
    Var_Ctr[2]=false;
    Var_Ctr[3]=false;

    AMIE.setIdleMode(IdleMode.kBrake);
    TMIE.setIdleMode(IdleMode.kBrake);
    AMSE.setIdleMode(IdleMode.kBrake);
    TMSE.setIdleMode(IdleMode.kBrake);
    AMSD.setIdleMode(IdleMode.kBrake);
    TMSD.setIdleMode(IdleMode.kBrake);
    AMID.setIdleMode(IdleMode.kBrake);
    TMID.setIdleMode(IdleMode.kBrake);
    INC.setIdleMode(IdleMode.kBrake);
    ITKP.setIdleMode(IdleMode.kBrake);
    ITKDS.setIdleMode(IdleMode.kBrake);
    ITKDI.setIdleMode(IdleMode.kBrake);

  }

  @Override
  public void teleopPeriodic() 
  {
    VMotor = controller.getRawAxis(3);
    VTransEixo = controller.getRawAxis(4);
    Direcao = Math.toDegrees(Math.atan2(controller.getRawAxis(0), controller.getRawAxis(1)));
    Translacao = Math.toDegrees(Math.atan2(controller.getRawAxis(4), controller.getRawAxis(5)));

    Analog_Stimu[0] = Analog_Stimulated(4, 5, 0.15);
    Analog_Stimu[1] = Analog_Stimulated(0, 1, 0.15);
    BTN_CTR[0] = controller.getAButton();

    BTN_CTR[1] = controller.getLeftBumper();
    BTN_CTR[2] = controller.getRightBumper();
  }

  @Override
  public void disabledInit() 
  {
    TMIE.setInverted(false);
    TMID.setInverted(false);
    TMSE.setInverted(false);
    TMSD.setInverted(false);
  }

  @Override
  public void disabledPeriodic() 
  {
    AMIE.setIdleMode(IdleMode.kCoast);
    TMIE.setIdleMode(IdleMode.kCoast);
    AMSE.setIdleMode(IdleMode.kCoast);
    TMSE.setIdleMode(IdleMode.kCoast);
    AMSD.setIdleMode(IdleMode.kCoast);
    TMSD.setIdleMode(IdleMode.kCoast);
    AMID.setIdleMode(IdleMode.kCoast);
    TMID.setIdleMode(IdleMode.kCoast);
    INC.setIdleMode(IdleMode.kCoast);

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
