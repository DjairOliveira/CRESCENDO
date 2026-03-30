package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    XboxController CoDriver_Controller;
    private CANSparkMax INC;
    private CANSparkMax SUSP;
    
    private CANSparkMax ITKP;
    private CANSparkMax ITKDS;
    private CANSparkMax ITKDI;

    private NetworkTable LimeTable;
    private int PipelineSelect = 1;
    private int LEDMODE=1;

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
    DigitalInput FCINC;
    DigitalInput FCSUSP;
    DigitalInput FCITK;

    double ITKP_Current=0;

    public boolean Modo_Coletar_Down=false, Modo_Coletar_Up=false;
    boolean Deposita_Amp=false;
    boolean Warning_Benga=false;
    boolean Warning_Suspensao=false;
  
    boolean Modo_AlinhamentoITK=false;

    double V_Inc = 0;
    double V_Susp=0;
    double VeloDisp=0, VeloColeta=0;

    boolean Ctr=false, Coletou=false;

    int cont1=0, cont2=0, cont3=0, Mult=0;
    boolean SUSP_Descanso = false;

    private boolean[] Var_Ctr = new boolean[10];

    double TargetX = 0;
    double TargetY = 0;
    double TargetArea = 0;

    boolean Auto_Lime=false, ITK_Down=false, ITK_Up=false, Disparo_Boost=false;

    boolean Nota_Entrando=false;
    boolean Shift_Posi=false;
    boolean Depositar=false;

    public Intake() {

      CoDriver_Controller = new XboxController(1);

      FCINC = new DigitalInput(0);
      FCSUSP = new DigitalInput(1);
      FCITK = new DigitalInput(2);

      INC = new CANSparkMax(9,MotorType.kBrushless);
      SUSP = new CANSparkMax(10,MotorType.kBrushless);
      
      ITKP = new CANSparkMax(13,MotorType.kBrushless);
      ITKDS = new CANSparkMax(11,MotorType.kBrushless);
      ITKDI = new CANSparkMax(12,MotorType.kBrushless);

      INC.getEncoder().setPosition(0);
      SUSP.getEncoder().setPosition(0);

      LimeTable = NetworkTableInstance.getDefault().getTable("limelight");

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

      Disparo_Boost=false;
    }
    public void Inclinacao()
    {
      if(CoDriver_Controller.getRawAxis(5)>=0.15 || (CoDriver_Controller.getRawAxis(5)<=-0.15  && Warning_Suspensao==true))
      {
        V_Susp = CoDriver_Controller.getRawAxis(5)*(-1)/7;
      }
      else
      {
        V_Susp=0;
      }
      if(SUSP.getEncoder().getPosition() >= 17 || SUSP.getEncoder().getPosition() <= 2)
      {
        V_Susp/=2;
      }

      if(Modo_AlinhamentoITK==false)
      {
        if((CoDriver_Controller.getPOV()==0 || ITK_Up==true) && Warning_Benga==true)
        {
          V_Inc=1;
        }
        if(CoDriver_Controller.getPOV()==180 || ITK_Down==true)
        {
          V_Inc=-1;
        }
        if(CoDriver_Controller.getPOV()==(-1) && Modo_Coletar_Down==false)
        {
          V_Inc=0;
        }
      }
      if(INC.getEncoder().getPosition() >= 438 || INC.getEncoder().getPosition() <= 5)
      {
        V_Inc/=3;
      }
    }
    public void Protecoes()
    {
      if(INC.getEncoder().getPosition() >= 447 && V_Inc > 0)      // Proteção de avanço do Benga
      {
        V_Inc=0;
      }

      if(FCINC.get()==true && V_Inc <= 0)      // Proteção de Recuo do Benga
      {
        Warning_Benga=true;
        INC.getEncoder().setPosition(0);
        V_Inc=0;
      }

      if(SUSP.getEncoder().getPosition() <= 5 && Warning_Suspensao==true)
      {
        SUSP_Descanso=true;
      }
      else
      {
        SUSP_Descanso=false;
      }

      if(SUSP_Descanso==false)
      {
        VeloDisp=0;
      }

      if(SUSP.getEncoder().getPosition() >= 18.5 && V_Susp > 0)      // Proteção de avanço do Suspensão
      {
        V_Susp=0;
      }

      if(FCSUSP.get()==true && V_Susp <= 0)      // Proteção de Recuo do Suspensão
      {
        Warning_Suspensao=true;
        SUSP.getEncoder().setPosition(0);
        V_Susp=0;
      }
    }
    public void Modo_Coleta(double Velocidade_Coleta, double Current_ITK)
    {
      if(CoDriver_Controller.getAButtonPressed()==true) //&& Coletou==false
      {
        Modo_Coletar_Down = !Modo_Coletar_Down;
      }
      if(CoDriver_Controller.getYButtonPressed()==true)
      {
        Modo_Coletar_Up = true;
      }
      if(Modo_Coletar_Up==false && Coletou==true)
      {
        V_Inc=1;
      }

      if(Modo_Coletar_Down==true)
      {
        Modo_Coletar_Up=false;
       
        if(Coletou==false)
        {
          V_Inc=-1;
          if(FCITK.get()==false && Nota_Entrando==false)
          {
            VeloColeta=Velocidade_Coleta;
          }
          if(FCITK.get()==true && Nota_Entrando==false)
          {
            CoDriver_Controller.setRumble(RumbleType.kBothRumble, 0.2);
            VeloColeta=0;
            cont1=0;
            Coletou=true;
            Nota_Entrando=true;
            Var_Ctr[9]=false;
          }
          if(Coletou==true && cont1<20)
          {
            cont1++;
            if(cont1>=20) CoDriver_Controller.setRumble(RumbleType.kBothRumble, 0);
            Modo_Coletar_Down = false;
          }
        }
        else Modo_Coletar_Down = false;
      }

      if(Modo_Coletar_Up==true)
      {
        Modo_Coletar_Down=false;
        if(Coletou==false)
        {
          if(FCITK.get()==false && Nota_Entrando==false)
          {
            VeloColeta=Velocidade_Coleta;
          }
          if(FCITK.get()==true && Nota_Entrando==false)
          {
            CoDriver_Controller.setRumble(RumbleType.kBothRumble, 0.2);
            VeloColeta=0;
            cont1=0;
            Coletou=true;
            Nota_Entrando=true;
            Var_Ctr[9]=false;
          }
          if(Coletou==true && cont1<20)
          {
            cont1++;
            if(cont1>=20) CoDriver_Controller.setRumble(RumbleType.kBothRumble, 0);
          }
        }
      }

      if(Modo_Coletar_Down==false && Coletou==false)
      {
        V_Inc=0;
        VeloColeta=0;
      }
    }
    public void Modo_Deposito(double VDeposito_Speaker, double VDeposito_Amp)
    {
        if(CoDriver_Controller.getAButtonPressed()==true)
        {
          if(Coletou==true && Nota_Entrando==true && Var_Ctr[9]==false)
          {
            cont2=0;
            if(INC.getEncoder().getPosition() >= 400) VeloDisp=VDeposito_Amp;     // 0.15
            if(INC.getEncoder().getPosition() < 400)
            {
              VeloDisp=VDeposito_Speaker;
              if(CoDriver_Controller.getRightBumper()==true || Disparo_Boost==true)
              {
                VeloDisp = VDeposito_Speaker+0.35;
              }
            }
            Depositar=true;
            Var_Ctr[9]=true;
          }
          }

        if(Depositar==true)
        {
          if(VeloDisp==VDeposito_Speaker || (VeloDisp==VDeposito_Speaker+0.35))
          {
            if(cont2<=40) cont2++;
            if(cont2>30)
            {
             VeloColeta=1;
             Deposita_Amp=false;
            }
            if(cont2>40)
            {
              CoDriver_Controller.setRumble(RumbleType.kBothRumble, 0);
              VeloColeta=0;
              VeloDisp=0;
              Coletou=false;
              Nota_Entrando=false;
              Depositar=false;
            }
          }
          if(VeloDisp==VDeposito_Amp)
          {
            if(cont2<=100) cont2++;
            if(cont2>15)
            {
             VeloColeta=1;
             Deposita_Amp=false;
            }
            if(cont2>30)
            {
              CoDriver_Controller.setRumble(RumbleType.kBothRumble, 0);
              VeloColeta=0;
              VeloDisp=0;
              Coletou=false;
              Nota_Entrando=false;
              Depositar=false;
            }
          }
          
        }
    }
    public void Position_Amplificador()
    {
      if(CoDriver_Controller.getXButton()==true && Var_Ctr[5]==false)
      {
        Deposita_Amp=!Deposita_Amp;
        Var_Ctr[5]=true;
      }
      if(CoDriver_Controller.getXButton()==false && Var_Ctr[5]==true)
      {
        Var_Ctr[5]=false;
      }
      if(Deposita_Amp==true && Warning_Benga==true)
      {
        V_Inc=1;
        Var_Ctr[8]=false;
      }
      if(Deposita_Amp==false && V_Inc==1 && Var_Ctr[8]==false)
      {
        V_Inc=0;
        Var_Ctr[8]=true;
      }
    }
    public void setAllTMotorsOutPut(double Value)
    {
      INC.set(V_Inc*Value);
      SUSP.set(V_Susp*Value);

      ITKP.set(VeloColeta*Value);
      ITKDS.set(VeloDisp*Value);
      ITKDI.set(VeloDisp*Value);
    }
    public void setIncliIdleMode(IdleMode Idle)
    {
      INC.setIdleMode(Idle);
      SUSP.setIdleMode(Idle);
    }
    public void AutoAlignmentLime(double TargetY_Deslocado)
    {
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

      LimeTable.getEntry("pipeline").setNumber(PipelineSelect);  // Define o valor da pipeline desejada
      LimeTable.getEntry("ledMode").setNumber(LEDMODE);          // Modo do Led definido para 1

      TargetX = LimeTable.getEntry("tx").getDouble(0.0);
      TargetY = LimeTable.getEntry("ty").getDouble(0.0);
      TargetArea = LimeTable.getEntry("ta").getDouble(0.0);

      if(Modo_AlinhamentoITK==true && TargetY!=0)
      {
        if(TargetY>(5+TargetY_Deslocado)) V_Inc=-1;
        if(TargetY<((-5) + TargetY_Deslocado)) V_Inc=1;
        if(TargetY>(0.5 + TargetY_Deslocado) && TargetY<=(5 + TargetY_Deslocado)) V_Inc=-0.5;
        if(TargetY>((-5)+TargetY_Deslocado) && TargetY<=((-0.5)+TargetY_Deslocado)) V_Inc=0.5;
        if(TargetY>((-0.5)+TargetY_Deslocado) && TargetY<=((0.5)+TargetY_Deslocado)) V_Inc=0;
      }
    }
    public void Shift_position(double REF, double P, double OutputRange, double Position_Range)
    {
      // if(Modo_Coletar==false && Warning_Benga==true)
      // {
      //   if()
      // }
      if(CoDriver_Controller.getXButtonPressed()==true)
      {
        Shift_Posi=!Shift_Posi;
      }

      if(Shift_Posi==true && Warning_Benga)
      {
          V_Inc = (REF-INC.getEncoder().getPosition())*P;
          
          if(V_Inc < Position_Range && V_Inc > -Position_Range) V_Inc = 0;
          if(V_Inc > OutputRange) V_Inc = OutputRange;
          if(V_Inc < OutputRange*(-1)) V_Inc = OutputRange*(-1);
      }
    }

    public boolean getColetou()
    {
      return Coletou;
    }

    public double getINCPosition()
    {
      return INC.getEncoder().getPosition();
    }

    public void SetColetar()
    {
      Modo_Coletar_Down=true;
    }

    public void setAutoLime(boolean status)
    {
      Auto_Lime=status;
    }

    public void setITKDown(boolean status)
    {
      ITK_Down=status;
    }

    public void setITKUp(boolean status)
    {
      ITK_Up=status;
    }

    public void SetDisparoBoost(boolean status)
    {
      Disparo_Boost=status;
    }

/*
    public void setPIDITK(double REF)
    {
      INC.getPIDController().setReference(REF, ControlType.kPosition);
    }
    public void SetRefInclination(double REF)
    {
      INC.getPIDController().setReference(REF ,ControlType.kPosition);;
    }
*/
    public void AutoSetITK()
    {
      Modo_Coletar_Down=false;
      Coletou=true;
      cont1=61;
    }
    public void SetIdleMode()
    {
      INC.setIdleMode(IdleMode.kBrake);
      SUSP.setIdleMode(IdleMode.kBrake);
      ITKP.setIdleMode(IdleMode.kBrake);
    }

    public void Exibi()
    {
      SmartDashboard.putNumber("Cont1", cont1);
      SmartDashboard.putNumber("Cont2", cont2);
      SmartDashboard.putNumber("Cont3", cont3);
      SmartDashboard.putBoolean("Modo CLT", Modo_Coletar_Down);
      SmartDashboard.putNumber("VeloColeta", VeloColeta);
      SmartDashboard.putBoolean("Coletou", Coletou);
      SmartDashboard.putNumber("VeloDisp", VeloDisp);
      SmartDashboard.putNumber("Velo_Inc", V_Inc);
      SmartDashboard.putNumber("ITKDI", ITKDI.getOutputCurrent());
      SmartDashboard.putNumber("ITKDS", ITKDS.getOutputCurrent());
      SmartDashboard.putNumber("ITKP", ITKP.getOutputCurrent());
      SmartDashboard.putNumber("INC", INC.getEncoder().getPosition());
      SmartDashboard.putBoolean("FC 1", FCINC.get());
      SmartDashboard.putBoolean("Benga", Warning_Suspensao);
      SmartDashboard.putBoolean("Coletar Down", Modo_Coletar_Down);
      SmartDashboard.putBoolean("Coletar Up", Modo_Coletar_Up);

    }

    @Override
    public void periodic() {
      if(CoDriver_Controller.getLeftBumper() || Auto_Lime==true) Modo_AlinhamentoITK=true;
      else
      {
        Modo_AlinhamentoITK=false;
      }

      Modo_Coleta(0.4, 7.5);
      Modo_Deposito(0.55, 0.2);   // 0.85

      if(Shift_Posi==false)
      {
          Inclinacao();
      }
      //Position_Amplificador();
      Shift_position(112.7, 0.3, 1, 0.2);
      if(Shift_Posi==false) AutoAlignmentLime(-10);
      SetIdleMode();
      Protecoes();
      setAllTMotorsOutPut(1);
    }

}
