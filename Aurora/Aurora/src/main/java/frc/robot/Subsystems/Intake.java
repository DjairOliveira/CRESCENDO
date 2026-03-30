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
    private static final String Pipe3  = "3";
    private static final String Pipe4  = "4";
    private static final String Pipe5  = "5";
    private static final String Pipe6  = "6";
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

    // public boolean Modo_Coletar_Down=false, Modo_Coletar_Up=false;
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
    boolean Shift_Perimetro = false;
    boolean Depositar=false;
    boolean Auto_Disparar=false;

    int Modo_Coletador=0;

    double TargetTy_Anterior=0, TargetTx_Anterior=0;

    boolean Protect_Auto=false;

    boolean Auto_Mode=false;

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

      m_Pipe.setDefaultOption("PIPE: BLUE 1X_", Pipe0);
      m_Pipe.addOption("PIPE: BLUE 1X", Pipe1);
      m_Pipe.addOption("PIPE: BLUE 2X", Pipe2);
      m_Pipe.addOption("PIPE: BLUE 3X", Pipe3);
      m_Pipe.addOption("PIPE: RED 1X", Pipe4);
      m_Pipe.addOption("PIPE: RED 2X", Pipe5);
      m_Pipe.addOption("PIPE: RED 3X", Pipe6);
      SmartDashboard.putData("PIPELINE SELECT:", m_Pipe);

      SmartDashboard.putData(CommandScheduler.getInstance());
      
      m_LimeLED.setDefaultOption("MODO: Default", LimeLED0);
      m_LimeLED.addOption("MODO: Off", LimeLED1);
      m_LimeLED.addOption("MODO: On", LimeLED2);
      m_LimeLED.addOption("MODO: On Process", LimeLED3);
      SmartDashboard.putData("LIMELED MODE:", m_LimeLED);
      SmartDashboard.putData(CommandScheduler.getInstance());
      PipelineSelect=1;
    }

    public double PID_JP(double Position, double Position_Desejada, double P, double OutputRange, double Position_Range)
    {
        double Value;
        Value = ((Position_Desejada)-(Position))*P;
          
        if((Value < Position_Range) && (Value > -Position_Range)) 
        {
          Value = 0;
        }
        if(Value > OutputRange) Value = OutputRange;
        if(Value < OutputRange*(-1)) Value = OutputRange*(-1);
        return Value;
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

      if(Modo_AlinhamentoITK==false && Modo_Coletador==0)
      {
        if((CoDriver_Controller.getPOV()==0 || ITK_Up==true) && Warning_Benga==true)
        {
          V_Inc=1;
        }
        if(CoDriver_Controller.getPOV()==180 || ITK_Down==true)
        {
          V_Inc=-1;
        }
        if(CoDriver_Controller.getPOV()==(-1) && ITK_Down==false && ITK_Up==false)
        {
          V_Inc=0;
        }
      }
      if((INC.getEncoder().getPosition() >= 438 || INC.getEncoder().getPosition() <= 5) && Protect_Auto==false)
      {
        V_Inc/=2;
      }
    }
    public void Protecoes()
    {
      if(INC.getEncoder().getPosition() >= 457 && V_Inc > 0)      // Proteção de avanço do Benga
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
        if(Modo_Coletador==0) Modo_Coletador=1;
        else{
            Modo_Coletador=0;
        }
      }
      if(CoDriver_Controller.getYButtonPressed()==true && Coletou==false)
      {
        if(Modo_Coletador==0) Modo_Coletador=2;
        else{
            Modo_Coletador=0;
        }
      }

      if(Modo_Coletador==1)
      {
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
            Modo_Coletador = 0;
          }
        }
        else Modo_Coletador = 0;
        }
        
      if(Modo_Coletador==2)
      {
        if(Coletou==false)
        {
          V_Inc=PID_JP(INC.getEncoder().getPosition(), 375.3, 0.3, 1, 0.5);

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
            Modo_Coletador = 0;
          }
        }
        else Modo_Coletador = 0;
      }

        if(Coletou==true && cont1<20)
        {
            cont1++;
            if(cont1>=20) CoDriver_Controller.setRumble(RumbleType.kBothRumble, 0);
        }

      if(Modo_Coletador==0 && Coletou==false)
      {
        V_Inc=0;
        VeloColeta=0;
      }
    }
    public void Modo_Deposito(double VDeposito_Speaker, double VDeposito_Amp)
    {
        if(CoDriver_Controller.getAButton()==true || Auto_Disparar==true)
        {
          if(Coletou==true && Nota_Entrando==true && Var_Ctr[9]==false)
          {
            cont2=0;
            if(INC.getEncoder().getPosition() >= 360) VeloDisp=VDeposito_Amp;     // 0.15
            if(INC.getEncoder().getPosition() < 360)
            {
              VeloDisp=VDeposito_Speaker;
                if(INC.getEncoder().getPosition()<360) //(INC.getEncoder().getPosition()>=86 &&  ///400
                {
                  VeloDisp = (VDeposito_Speaker + (INC.getEncoder().getPosition() / 400));
                  if (VeloDisp > 1) VeloDisp = 1;
                }
            }
            Depositar=true;
            Var_Ctr[9]=true;
          }
        }

        if(Depositar==true)
        {
          if(VeloDisp != VDeposito_Amp)
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
            if(cont2<=30) cont2++;
            if(cont2>15)  // 15
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

      // if(PipelineSelect==1 && TargetY==0)
      // {
      //   PipelineSelect=2;
      // }
      // if(PipelineSelect==2 && Depositar==true)
      // {
      //   PipelineSelect=1;
      // }

      // if(PipelineSelect==4 && TargetY==0)
      // {
      //   PipelineSelect=5;
      // }
      // if(PipelineSelect==5 && Depositar==true)
      // {
      //   PipelineSelect=4;
      // }

      if(TargetX!=0 && TargetY!=0)
      {
        double REF_Y = (TargetY_Deslocado - (INC.getEncoder().getPosition() / 100));
        V_Inc = PID_JP(TargetY, REF_Y, 0.2, 1, 0.3);
      }
    }

    public void Shift_position(double REF, double P, double OutputRange, double Position_Range)
    {
      if(CoDriver_Controller.getXButtonPressed()==true)
      {
        Shift_Posi=!Shift_Posi;
        Modo_Coletador=0;
      }

      if(Shift_Posi==true && Warning_Benga)
      {
          V_Inc = (REF-INC.getEncoder().getPosition())*P;
          
          if(V_Inc < Position_Range && V_Inc > -Position_Range) 
          {
            V_Inc = 0;
            Shift_Posi=false;
          }
          if(V_Inc > OutputRange) V_Inc = OutputRange;
          if(V_Inc < OutputRange*(-1)) V_Inc = OutputRange*(-1);
      }
    }

    public void Shift_Perimetro()
    {
      if(CoDriver_Controller.getRawButtonPressed(6)==true)
      {
        Shift_Perimetro=!Shift_Perimetro;
      }
      if(Shift_Perimetro==true)
      {
        V_Inc = PID_JP(INC.getEncoder().getPosition(), 267.71, 0.3, 1, 0.1);
        Shift_Perimetro=false;
      }

    }

    public boolean getColetou()
    {
      return Coletou;
    }

    public boolean getFCINC()
    {
      return FCINC.get();
    }

    public void setPipeline(int Pipe)
    {
        PipelineSelect=Pipe;
    }
    
    public double getTargetX()
    {
      return TargetX;
    }

    public double getINCPosition()
    {
      return INC.getEncoder().getPosition();
    }

    public void SetColetar()
    {
      Modo_Coletador=1;
    }

    public void setAutoLime(boolean status)
    {
      Auto_Lime=status;
    }

    public boolean getControllerAling()
    {
      return CoDriver_Controller.getLeftBumper();
    }

    public void setAutoProtect(boolean status)
    {
      Protect_Auto=status;
    }

    public void setITKDown(boolean status)
    {
      ITK_Down=status;
      Modo_Coletador=0;
      Modo_AlinhamentoITK=false;
    }

    public void setITKUp(boolean status)
    {
      ITK_Up=status;
    }

    public void SetDisparo(boolean status)
    {
      Auto_Disparar=status;
    }
    public void AutoSetITK()
    {
      Nota_Entrando=true;
      Var_Ctr[9]=false;
      // Modo_Coletador=0;
      Coletou=true;
    }
    public void Modo_Coletar(int Mode)
    {
      Modo_Coletador=Mode;
    }
    public void SetIdleMode()
    {
      INC.setIdleMode(IdleMode.kBrake);
      SUSP.setIdleMode(IdleMode.kBrake);
      ITKP.setIdleMode(IdleMode.kBrake);
    }
    public double getTargetY()
    {
      return TargetY;
    }
    public double getCurrentITKP()
    {
      return ITKP.getOutputCurrent();
    }
    public boolean getDepositar()
    {
      return Depositar;
    }
    public void setAuto_Mode(boolean status)
    {
      Auto_Mode=status;
    }

    public void Disable_Auto()
    {
      ITK_Up=false;
      ITK_Down=false;
      Protect_Auto=false;
      Auto_Disparar=false;
      Auto_Lime=false;
      Auto_Mode=false;
      PipelineSelect=1;
    }
    
    public void Exibi()
    {
        // SmartDashboard.putNumber("Cont1", cont1);
        // SmartDashboard.putNumber("Cont2", cont2);
        // SmartDashboard.putNumber("Cont3", cont3);
        // SmartDashboard.putNumber("VeloColeta", VeloColeta);
        // SmartDashboard.putBoolean("Coletou", Coletou);
        // SmartDashboard.putNumber("VeloDisp", VeloDisp);
        // SmartDashboard.putNumber("Velo_Inc", V_Inc);
        // SmartDashboard.putNumber("ITKDI", ITKDI.getOutputCurrent());
        // SmartDashboard.putNumber("ITKDS", ITKDS.getOutputCurrent());
        // SmartDashboard.putNumber("ITKP", ITKP.getOutputCurrent());
        // SmartDashboard.putNumber("INC", INC.getEncoder().getPosition());
        SmartDashboard.putBoolean("FC 1", FCINC.get());
        SmartDashboard.putBoolean("Benga", Warning_Suspensao);
        // SmartDashboard.putNumber("Modo Coleta", Modo_Coletador);
        // SmartDashboard.putBoolean("Depositar", Depositar);
        // SmartDashboard.putBoolean("Nota_Entrando", Nota_Entrando);
        // SmartDashboard.putNumber("TargetTy_Anterior", TargetTy_Anterior);
        // SmartDashboard.putNumber("TargetTx_Anterior", TargetTx_Anterior);
        SmartDashboard.putNumber("TargetTx", TargetX);
        SmartDashboard.putNumber("TargetTy", TargetY);
        SmartDashboard.putNumber("Pipeline", PipelineSelect);
        
    }

    @Override
    public void periodic() {
        if(CoDriver_Controller.getLeftBumper() || Auto_Lime==true) Modo_AlinhamentoITK=true;
        else
        {
          Modo_AlinhamentoITK=false;
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
          case "3":
            PipelineSelect=3;
            break;
          case "4":
            PipelineSelect=4;
            break;
          case "5":
            PipelineSelect=5;
            break;
          case "6":
            PipelineSelect=6;
            break;

          default:
            PipelineSelect=1;
            break;
        }
        LimeTable.getEntry("pipeline").setNumber(PipelineSelect);  // Define o valor da pipeline desejada
        LimeTable.getEntry("ledMode").setNumber(LEDMODE);          // Modo do Led definido para 1
        
        TargetX = LimeTable.getEntry("tx").getDouble(0.0);
        TargetY = LimeTable.getEntry("ty").getDouble(0.0);
        TargetArea = LimeTable.getEntry("ta").getDouble(0.0);

        if(Warning_Benga==true) Shift_Perimetro();

        if(Shift_Posi==false) Modo_Coleta(0.4, 7.5);
        Modo_Deposito(0.6, 0.5);   // 0.85   055  0.27

        if(Shift_Posi==false) Inclinacao();
        Shift_position(112.7, 0.3, 1, 0.2);
        if(Shift_Posi==false && Modo_AlinhamentoITK==true) AutoAlignmentLime(-4);  //-10
        SetIdleMode();
        if(LimeTable.getEntry("tx").getDouble(0.0)!=0 && LimeTable.getEntry("ty").getDouble(0.0)!=0) 
        {
            TargetTx_Anterior = LimeTable.getEntry("tx").getDouble(0.0);
            TargetTy_Anterior = LimeTable.getEntry("ty").getDouble(0.0);
        }
        
        Protecoes();
        setAllTMotorsOutPut(1);
    }
}
