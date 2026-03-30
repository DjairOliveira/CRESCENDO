package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swervedrive extends SubsystemBase{

    private CANSparkMax TMSE;                         // TRAÇÃO MOTOR SUPERIOR ESQUERDO
    private CANSparkMax TMSD;                         // TRAÇÃO MOTOR SUPERIOR DIREITO
    private CANSparkMax TMIE;                         // TRAÇÃO MOTOR INFERIOR ESQUERDO
    private CANSparkMax TMID;                         // TRAÇÃO MOTOR INFERIOR DIREITO

    private CANSparkMax AMSE;                         // ANGULO MOTOR SUPERIOR ESQUERDO
    private CANSparkMax AMSD;                         // ANGULO MOTOR SUPERIOR DIREITO
    private CANSparkMax AMIE;                         // ANGULO MOTOR INFERIOR ESQUERDO
    private CANSparkMax AMID;                         // ANGULO MOTOR INFERIOR DIREITO

    private CANcoder CANC_SE;
    private CANcoder CANC_SD;
    private CANcoder CANC_IE;
    private CANcoder CANC_ID;

    private XboxController controller;

    private Pigeon2 Pigeon;

    private Intake Itk;

    private double Ref_Encoder_SD = 16.417020784085121;     // Referência do Encoder: 360°/21.38 = 16.83816651075772 (21.38 = 360° do Encoder)  21,928461
    private double Ref_Encoder_SE = 16.744341807830770;      // 21,4998
    private double Ref_Encoder_IE = 16.762829431505402;     // 21,476088
    private double Ref_Encoder_ID = 16.417020784085121;     //21,928461

    private double Offset_CCSD = -96.92; // 263.08 
    private double Offset_CCSE = -22.37; // 337.63
    private double Offset_CCIE = -128.68; // 231.32
    private double Offset_CCID = -144.05; // 215.95

    private double Range_Modifier = 30;

    private boolean Var_Ctr[] = new boolean[9];

    private int Cont[] = new int[2];
    private int Auto_Cont[] = new int[10];

    int Passo=0;

    public Swervedrive() {

        Itk = new Intake();
        
        Var_Ctr[0] = false;
        Var_Ctr[1] = false;
        Var_Ctr[2] = false;
        Var_Ctr[3] = false;
        Cont[0] = 0;
        Cont[1] = 0;
        controller = new XboxController(0);

        Pigeon = new Pigeon2(14);
        
        TMIE = new CANSparkMax(7,MotorType.kBrushless);
        AMIE = new CANSparkMax(8,MotorType.kBrushless);
        AMIE.setInverted(true);
        CANC_IE = new CANcoder(18);

        TMID = new CANSparkMax(5,MotorType.kBrushless);
        AMID = new CANSparkMax(6,MotorType.kBrushless);
        AMID.setInverted(true);
        CANC_ID = new CANcoder(17);

        TMSE = new CANSparkMax(1,MotorType.kBrushless);
        AMSE = new CANSparkMax(2,MotorType.kBrushless);
        AMSE.setInverted(true);
        CANC_SE = new CANcoder(15);

        TMSD = new CANSparkMax(3,MotorType.kBrushless);
        AMSD = new CANSparkMax(4,MotorType.kBrushless);
        AMSD.setInverted(true);
        CANC_SD = new CANcoder(16);

        SPARKMAX_CONFIG_TRACAO(TMIE);
        SPARKMAX_CONFIG_TRACAO(TMID);
        SPARKMAX_CONFIG_TRACAO(TMSD);
        SPARKMAX_CONFIG_TRACAO(TMSE);

        SPARKMAX_CONFIG_ANGULO(AMIE, Ref_Encoder_IE, 0.3);
        SPARKMAX_CONFIG_ANGULO(AMID, Ref_Encoder_ID, 0.3);
        SPARKMAX_CONFIG_ANGULO(AMSD, Ref_Encoder_SD, 0.3);
        SPARKMAX_CONFIG_ANGULO(AMSE, Ref_Encoder_SE, 0.3);

        setTEncPosition(0);

        Pigeon.setYaw(0);
    }

    public double PID_Wrapping(double Position, double Position_Desejada, double P, double OutputRange, double Position_Range, double Range_Warapping)
    {
        double Value;

        if(((Position_Desejada - Position)>(Range_Warapping/2)) || (Position_Desejada - Position)<(-Range_Warapping/2))
        {
          Value = (Position - Position_Desejada) * P;
        }
        else
        {
          Value = (Position_Desejada-Position)*P;
        }

        if((Position_Desejada-Position) < Position_Range && (Position_Desejada-Position) > -Position_Range) 
        {
          Value = 0;
        }
        if(Value > OutputRange) Value = OutputRange;
        if(Value < OutputRange*(-1)) Value = OutputRange*(-1);
        return Value;
    }
    public void ResetYaw() {
        Pigeon.setYaw(0);
    }

    public void SwerveSparksResFacDefault() {
        TMIE.restoreFactoryDefaults();
        TMID.restoreFactoryDefaults();
        TMSD.restoreFactoryDefaults();
        TMSE.restoreFactoryDefaults();

        AMIE.restoreFactoryDefaults();
        AMID.restoreFactoryDefaults();
        AMSD.restoreFactoryDefaults();
        AMSE.restoreFactoryDefaults();
    }

    public static void SPARKMAX_CONFIG_TRACAO(CANSparkMax Motor)                        // Configura o PID e range de saida para o motor Alvo
    {
        
        Motor.getPIDController().setP(1);
        Motor.getPIDController().setI(0);
        Motor.getPIDController().setD(0);
        Motor.getPIDController().setOutputRange(-1, 1);
        Motor.setInverted(false);
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
    public void setPIDconfig(CANSparkMax motor, double P_value, double OutputRange) {
        motor.getPIDController().setP(P_value);
        motor.getPIDController().setI(0);
        motor.getPIDController().setD(0);
        motor.getPIDController().setOutputRange(OutputRange*(-1), OutputRange);
    }

    public void setPIDRef4TMotors(double Reference) {
        TMIE.getPIDController().setReference(Reference, ControlType.kPosition);
        TMID.getPIDController().setReference(Reference, ControlType.kPosition);
        TMSD.getPIDController().setReference(Reference, ControlType.kPosition);
        TMSE.getPIDController().setReference(Reference, ControlType.kPosition);
    }

    public void setTEncPosition(double Position) {
        TMIE.getEncoder().setPosition(Position);
        TMID.getEncoder().setPosition(Position);
        TMSD.getEncoder().setPosition(Position);
        TMSE.getEncoder().setPosition(Position);
    }



    public void setCCABS4MotorEnc() {
        AMID.getEncoder().setPosition(getCCABS(CANC_ID, Offset_CCID)); // modo convencional
        AMIE.getEncoder().setPosition(getCCABS(CANC_IE, Offset_CCIE)); // somente seta os encoderes dos
        AMSD.getEncoder().setPosition(getCCABS(CANC_SD, Offset_CCSD)); // motores para a posição do 
        AMSE.getEncoder().setPosition(getCCABS(CANC_SE, Offset_CCSE)); // cancoder
    }

    public double getCCABS(CANcoder cancoder, double Offset) {
        double CCABS = ((cancoder.getAbsolutePosition().getValueAsDouble()*360) - Offset);
        if (CCABS < 0) CCABS *= (-1);
        CCABS %= 360;
        return CCABS;
    }

    public double getABSYaw() {
        Double ABSYaw = (Pigeon.getYaw().getValueAsDouble() * (-1));
        ABSYaw %= 360;
        if(ABSYaw<0) ABSYaw+=360;
        return ABSYaw;
    }

    public void setAllIdleMode(IdleMode mode) {
        AMIE.setIdleMode(mode);
        TMIE.setIdleMode(mode);
        AMSE.setIdleMode(mode);
        TMSE.setIdleMode(mode);
        AMSD.setIdleMode(mode);
        TMSD.setIdleMode(mode);
        AMID.setIdleMode(mode);
        TMID.setIdleMode(mode);
    }

    public boolean getJoyStimulated(int Axisx, int Axisy, double Range) {
        return (controller.getRawAxis(Axisx) <= (-Range) 
        || controller.getRawAxis(Axisx) >= Range 
        || controller.getRawAxis(Axisy) <= (-Range) 
        || controller.getRawAxis(Axisy) >= Range);
    }
    public double getJoy_degrees(int Axisx, int Axisy) { 
        return Math.toDegrees(
            Math.atan2(controller.getRawAxis(Axisx), 
            controller.getRawAxis(Axisy)));
    }

    public void Mov2Pose0() {
        AMIE.getPIDController().setReference(0, ControlType.kPosition);
        AMID.getPIDController().setReference(0, ControlType.kPosition);
        AMSE.getPIDController().setReference(0, ControlType.kPosition);
        AMSD.getPIDController().setReference(0, ControlType.kPosition);
        setAllTMotorsInvert(false);
    }

    public void setAllTMotorsOutPut(double SE, double SD, double ID, double IE) {
        TMIE.set(IE);
        TMID.set(ID);
        TMSE.set(SE);
        TMSD.set(SD);
    }
    public void setAllTMotorsOutPut(double Value) {
        TMIE.set(Value);
        TMID.set(Value);
        TMSE.set(Value);
        TMSD.set(Value);
    }
    public void setAllITKMotorsOutPut(double Value) {
        Itk.setAllTMotorsOutPut(Value);
    }
    public void setAllTMotorsInvert(boolean Invert) {
        TMIE.setInverted(Invert);
        TMID.setInverted(Invert);
        TMSE.setInverted(Invert);
        TMSD.setInverted(Invert);
    }

  
    public void setPIDConfigAngulo(double P_value, double OutputRange) {
        setPIDconfig(AMID, P_value, OutputRange);
        setPIDconfig(AMSD, P_value, OutputRange);
        setPIDconfig(AMIE, P_value, OutputRange);
        setPIDconfig(AMSE, P_value, OutputRange);
    }
     public void setPIDConfigTracao(double P_value, double OutputRange) {
        setPIDconfig(TMID, P_value, OutputRange);
        setPIDconfig(TMSD, P_value, OutputRange);
        setPIDconfig(TMIE, P_value, OutputRange);
        setPIDconfig(TMSE, P_value, OutputRange);
    }

    public void MovSpin() {
        AMID.getPIDController().setReference(315, ControlType.kPosition);
        AMIE.getPIDController().setReference(225, ControlType.kPosition);
        AMSD.getPIDController().setReference(45, ControlType.kPosition);
        AMSE.getPIDController().setReference(135, ControlType.kPosition);
        setAllTMotorsInvert(false);
        setAllTMotorsOutPut(controller.getRightX() / 4);
    }
     public void MovSpin(double Velocity) {
        AMID.getPIDController().setReference(315, ControlType.kPosition);
        AMIE.getPIDController().setReference(225, ControlType.kPosition);
        AMSD.getPIDController().setReference(45, ControlType.kPosition);
        AMSE.getPIDController().setReference(135, ControlType.kPosition);
        setAllTMotorsInvert(false);
        setAllTMotorsOutPut(Velocity);
    }

    public void MovDefault() {
        double REF = getABSYaw() + getJoy_degrees(0, 1);
        AMIE.getPIDController().setReference(REF ,ControlType.kPosition);
        AMID.getPIDController().setReference(REF ,ControlType.kPosition);
        AMSE.getPIDController().setReference(REF ,ControlType.kPosition);
        AMSD.getPIDController().setReference(REF ,ControlType.kPosition);
        setAllTMotorsOutPut(controller.getRightTriggerAxis());
    }

     public void MovDefault(double Angulo, double Distance) {
        double REF_Angulo = getABSYaw() + Angulo;
        AMIE.getPIDController().setReference(REF_Angulo ,ControlType.kPosition);
        AMID.getPIDController().setReference(REF_Angulo ,ControlType.kPosition);
        AMSE.getPIDController().setReference(REF_Angulo ,ControlType.kPosition);
        AMSD.getPIDController().setReference(REF_Angulo ,ControlType.kPosition);
        setPIDRef4TMotors(Distance);
    }

    public void MovMeuAmiiigo() {

        double REF_Default = getJoy_degrees(0, 1);
        double Modifier = ((controller.getRightX() * Range_Modifier) * (-1));
        double ModifierSin = (Math.sin(Math.toRadians(Modifier)));
        if (ModifierSin < 0) ModifierSin *= (-1);
        double TriggerRight = controller.getRightTriggerAxis();
        double TriggerRightModified = TriggerRight - (ModifierSin / 3);
        if (TriggerRightModified < 0) TriggerRightModified = 0;
        //double REF_Min = REF_Default - Range_Modifier;
        //double REF_Max = REF_Default + Range_Modifier;
        //double REF_D_Iverted = REF_Default + 180;

        //if (REF_D_Iverted > 180) REF_D_Iverted -= 360;

        //if (REF_Default < 0) {
        //    if (Modifier > REF_D_Iverted) Modifier -= 360;
        //}
        //else {
        //    if (Modifier < REF_D_Iverted) Modifier += 360;
        //}

        //double REF_Modified = ((REF_Default + Modifier) / 2);

        //if (REF_Modified < REF_Min) REF_Modified = REF_Min;
        //if (REF_Modified > REF_Max) REF_Modified = REF_Max;
        double REF_Modified = REF_Default + Modifier;
        double ABSYaw = getABSYaw();
        if (ABSYaw < 0) ABSYaw += 360;

        if (REF_Modified < (-180)) REF_Modified += 360;
        if (REF_Modified > 180) REF_Modified -= 360;

         if(((getJoy_degrees(0, 1)>=135 || getJoy_degrees(0, 1)<=-134.999)
          && (ABSYaw >= 315 || ABSYaw <= 44.999))
              || 
          ((getJoy_degrees(0, 1)>=(-135) && getJoy_degrees(0, 1)<=(-44.999))
          && (ABSYaw >= 225 && ABSYaw <= 314.999))
              || 
          ((getJoy_degrees(0, 1)>=45 && getJoy_degrees(0, 1)<=134.999)
          && (ABSYaw >= 45 && ABSYaw <= 134.999))
              || 
          ((getJoy_degrees(0, 1)>=(-45) && getJoy_degrees(0, 1)<=44.999)
          && (ABSYaw >= 135 && ABSYaw <= 224.999)));
          {
            AMIE.getPIDController().setReference(getABSYaw() + REF_Default ,ControlType.kPosition);
            AMID.getPIDController().setReference(getABSYaw() + REF_Default ,ControlType.kPosition);
            AMSE.getPIDController().setReference(getABSYaw() + REF_Modified ,ControlType.kPosition);
            AMSD.getPIDController().setReference(getABSYaw() + REF_Modified ,ControlType.kPosition);
            if (Modifier < 0) {
              setAllTMotorsOutPut(TriggerRight, TriggerRightModified, TriggerRightModified, TriggerRight);
            }
            else {
              setAllTMotorsOutPut(TriggerRightModified, TriggerRight, TriggerRight, TriggerRightModified);
            }
          }
      
          if(((getJoy_degrees(0, 1)>=135 || getJoy_degrees(0, 1)<=-134.999) 
          && (ABSYaw >= 45 && ABSYaw <= 134.999))
              || 
          ((getJoy_degrees(0, 1)>=(-135) && getJoy_degrees(0, 1)<=(-44.999))
          && (ABSYaw >= 315 || ABSYaw <= 44.999))
              || 
          ((getJoy_degrees(0, 1)>=45 && getJoy_degrees(0, 1)<=134.999)
          && (ABSYaw >= 135 && ABSYaw <= 224.999))
              || 
          ((getJoy_degrees(0, 1)>=(-45) && getJoy_degrees(0, 1)<=44.999)
          && (ABSYaw >= 225 && ABSYaw <= 314.999)))
          {
            AMIE.getPIDController().setReference(getABSYaw() + REF_Modified ,ControlType.kPosition);
            AMID.getPIDController().setReference(getABSYaw() + REF_Default ,ControlType.kPosition);
            AMSE.getPIDController().setReference(getABSYaw() + REF_Modified ,ControlType.kPosition);
            AMSD.getPIDController().setReference(getABSYaw() + REF_Default ,ControlType.kPosition);
            if (Modifier < 0) {
              setAllTMotorsOutPut(TriggerRightModified, TriggerRightModified, TriggerRight, TriggerRight);
            }
            else {
              setAllTMotorsOutPut(TriggerRight, TriggerRight, TriggerRightModified, TriggerRightModified);
            }
          }

          if(((getJoy_degrees(0, 1)>=135 || getJoy_degrees(0, 1)<=-134.999) 
          && (ABSYaw >= 135 && ABSYaw <= 224.999))
              || 
          ((getJoy_degrees(0, 1)>=(-135) && getJoy_degrees(0, 1)<=(-44.999))
          && (ABSYaw >= 45 && ABSYaw <= 134.999))
              || 
              ((getJoy_degrees(0, 1)>=45 && getJoy_degrees(0, 1)<=134.999)
          && (ABSYaw >= 225 && ABSYaw <= 314.999))
              || 
              ((getJoy_degrees(0, 1)>=(-45) && getJoy_degrees(0, 1)<=44.999)
          && (ABSYaw >= 315 || ABSYaw <= 44.999)))
          {
            AMIE.getPIDController().setReference(getABSYaw() + REF_Modified ,ControlType.kPosition);
            AMID.getPIDController().setReference(getABSYaw() + REF_Modified ,ControlType.kPosition);
            AMSE.getPIDController().setReference(getABSYaw() + REF_Default ,ControlType.kPosition);
            AMSD.getPIDController().setReference(getABSYaw() + REF_Default ,ControlType.kPosition);
            if (Modifier < 0) {
              setAllTMotorsOutPut(TriggerRightModified, TriggerRight, TriggerRight, TriggerRightModified);
            }
            else {
              setAllTMotorsOutPut(TriggerRight, TriggerRightModified, TriggerRightModified, TriggerRight);
            }
          }
          if(((getJoy_degrees(0, 1)>=135 || getJoy_degrees(0, 1)<=-134.999) 
          && (ABSYaw >= 225 && ABSYaw <= 314.999))
              || 
          ((getJoy_degrees(0, 1)>=(-135) && getJoy_degrees(0, 1)<=(-44.999))
          && (ABSYaw >= 135 && ABSYaw <= 224.999))
              || 
          ((getJoy_degrees(0, 1)>=45 && getJoy_degrees(0, 1)<=134.999)
          && (ABSYaw >= 315 || ABSYaw <= 44.999))
              || 
          ((getJoy_degrees(0, 1)>=(-45) && getJoy_degrees(0, 1)<=44.999)
          && (ABSYaw >= 45 && ABSYaw <= 134.999)))
          {
            AMIE.getPIDController().setReference(getABSYaw() + REF_Default ,ControlType.kPosition);
            AMID.getPIDController().setReference(getABSYaw() + REF_Modified ,ControlType.kPosition);
            AMSE.getPIDController().setReference(getABSYaw() + REF_Default ,ControlType.kPosition);
            AMSD.getPIDController().setReference(getABSYaw() + REF_Modified ,ControlType.kPosition);
            if (Modifier < 0) {
              setAllTMotorsOutPut(TriggerRight, TriggerRight, TriggerRightModified, TriggerRightModified);
            }
            else {
              setAllTMotorsOutPut(TriggerRightModified, TriggerRightModified, TriggerRight, TriggerRight);
            }
          }
        }

    public void MovBixuru()
    {
        double REF = (getABSYaw()+getJoy_degrees(0, 1));
        double REF_Invert = (getABSYaw()+getJoy_degrees(4, 5));

        if((getJoy_degrees(4, 5)>=135 || getJoy_degrees(4, 5)<=-134.999) && (getJoy_degrees(0, 1)>=135 || getJoy_degrees(0, 1)<=-134.999))
        {
          if(getABSYaw() >= 315 || getABSYaw() <= 44.999)
          {
            // MASE = (getABSYaw()+getJoy_degrees(4, 5));
            // MASD = (getABSYaw()+getJoy_degrees(4, 5));
            // MAID = (getABSYaw()+getJoy_degrees(0, 1));
            // MAIE = (getABSYaw()+getJoy_degrees(0, 1));

            AMIE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
          }
      
          if(getABSYaw()>=45 && getABSYaw()<=134.999)
          {
            AMIE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF ,ControlType.kPosition);
          }

          if(getABSYaw()>=135 && getABSYaw()<=224.999)
          {
            AMIE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF ,ControlType.kPosition);
          }
          if(getABSYaw()>=225 && getABSYaw()<=314.999)
          {
            AMIE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
          }
        }
        if((getJoy_degrees(4, 5)>=(-135) && getJoy_degrees(4, 5)<=(-44.999)) && (getJoy_degrees(0, 1)>=(-135) && getJoy_degrees(0, 1)<=(-44.999)))
        {
          if(getABSYaw()>=315 || getABSYaw()<=44.999)
          {
            AMIE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF ,ControlType.kPosition);
          }
      
          if(getABSYaw()>=45 && getABSYaw()<=134.999)
          {
            AMIE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF ,ControlType.kPosition);
          }

          if(getABSYaw()>=135 && getABSYaw()<=224.999)
          {
            AMIE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
          }
          if(getABSYaw()>=225 && getABSYaw()<=314.999)
          {
            AMIE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
          }
        }
        if((getJoy_degrees(4, 5)>=45 && getJoy_degrees(4, 5)<=134.999) && (getJoy_degrees(0, 1)>=45 && getJoy_degrees(0, 1)<=134.999))
        {
          if(getABSYaw()>=315 || getABSYaw()<=44.999)
          {
            AMIE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
          }
      
          if(getABSYaw()>=45 && getABSYaw()<=134.999)
          {
            AMIE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
          }

          if(getABSYaw()>=135 && getABSYaw()<=224.999)
          {
            AMIE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF ,ControlType.kPosition);
          }
          if(getABSYaw()>=225 && getABSYaw()<=314.999)
          {
            AMIE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF ,ControlType.kPosition);
          }
        }
        if((getJoy_degrees(4, 5)>=(-45) && getJoy_degrees(4, 5)<=44.999) && (getJoy_degrees(0, 1)>=(-45) && getJoy_degrees(0, 1)<=44.999))
        {
          if(getABSYaw()>=315 || getABSYaw()<=44.999)
          {
            AMIE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF ,ControlType.kPosition);
          }
      
          if(getABSYaw()>=45 && getABSYaw()<=134.999)
          {
            AMIE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
          }

          if(getABSYaw()>=135 && getABSYaw()<=224.999)
          {
            AMIE.getPIDController().setReference(REF ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
          }
          if(getABSYaw()>=225 && getABSYaw()<=314.999)
          {
            AMIE.getPIDController().setReference(REF_Invert  ,ControlType.kPosition);
            AMID.getPIDController().setReference(REF ,ControlType.kPosition);
            AMSE.getPIDController().setReference(REF_Invert ,ControlType.kPosition);
            AMSD.getPIDController().setReference(REF,ControlType.kPosition);
          }
        }
        setAllTMotorsOutPut(controller.getRightTriggerAxis());
    }

    public void noMov() {
        setAllTMotorsOutPut(0);
        AMIE.getPIDController().setReference(getABSYaw(), ControlType.kPosition);
        AMID.getPIDController().setReference(getABSYaw(), ControlType.kPosition);
        AMSE.getPIDController().setReference(getABSYaw(), ControlType.kPosition);
        AMSD.getPIDController().setReference(getABSYaw(), ControlType.kPosition);
    }

    public void printDebugvalues() {
        // SmartDashboard.putNumber("CANC_ID", getCCABS(CANC_ID, Offset_CCID));
        // SmartDashboard.putNumber("CANC_SD", getCCABS(CANC_SD, Offset_CCSD));
        // SmartDashboard.putNumber("CANC_SE", getCCABS(CANC_SE, Offset_CCSE));
        // SmartDashboard.putNumber("CANC_IE", getCCABS(CANC_IE, Offset_CCIE));
        // SmartDashboard.putNumber("CC_ID_Origin", ((getCCABS(CANC_ID, Offset_CCID) + Offset_CCID) % 360));
        // SmartDashboard.putNumber("CC_SD_Origin", ((getCCABS(CANC_SD, Offset_CCSD) + Offset_CCSD) % 360));
        // SmartDashboard.putNumber("CC_SE_Origin", ((getCCABS(CANC_SE, Offset_CCSE) + Offset_CCSE) % 360));
        // SmartDashboard.putNumber("CC_IE_Origin", ((getCCABS(CANC_IE, Offset_CCIE) + Offset_CCIE) % 360));
        // SmartDashboard.putNumber("Offset_CC_ID", Offset_CCID);
        // SmartDashboard.putNumber("Offset_CC_SD", Offset_CCSD);
        // SmartDashboard.putNumber("Offset_CC_SE", Offset_CCSE);
        // SmartDashboard.putNumber("Offset_CC_IE", Offset_CCIE);
        // SmartDashboard.putNumber("ENC_ID",AMID.getEncoder().getPosition());
        // SmartDashboard.putNumber("ENC_IE",AMIE.getEncoder().getPosition());
        // SmartDashboard.putNumber("ENC_SD",AMSD.getEncoder().getPosition());
        // SmartDashboard.putNumber("ENC_SE",AMSE.getEncoder().getPosition());  
        // SmartDashboard.putNumber("P_PID_ID", AMID.getPIDController().getP());
        // SmartDashboard.putBoolean("inverted", AMID.getInverted());
        // SmartDashboard.putBoolean("wrapping", AMID.getPIDController().getPositionPIDWrappingEnabled());
        // SmartDashboard.putNumber("Contador Auto", Cont[1]);
        SmartDashboard.putNumber("DistanceID T", TMID.getEncoder().getPosition());
        SmartDashboard.putNumber("DistanceIE T", TMIE.getEncoder().getPosition());
        SmartDashboard.putNumber("DistanceSD T", TMSD.getEncoder().getPosition());
        SmartDashboard.putNumber("DistanceSE T", TMSE.getEncoder().getPosition());
        SmartDashboard.putNumber("Passos", Passo);
        SmartDashboard.putNumber("getYAWABS", getABSYaw());
        Itk.Exibi();
    }

    public void zerador() {
        Cont[1] = 0;
    }

    public void MovTeleOp() {
        Itk.setAutoProtect(false);

         if(Itk.getControllerAling()==true)
         {
           MovSpin(Itk.PID_JP(-2, Itk.getTargetX(), 0.1, 0.1, 0.4));
           Var_Ctr[3] = true;
         }

        if (getJoyStimulated(0, 1, 0.15) && getJoyStimulated(4, 5, 0.15) && !Var_Ctr[3]) {
            MovMeuAmiiigo();
            Var_Ctr[0] = true;
        }
        if (getJoyStimulated(0, 1, 0.15) && !Var_Ctr[0]&& !Var_Ctr[3]) {
            MovDefault();
            Var_Ctr[1] = true;
        }
        if (getJoyStimulated(4, 5, 0.15) && !Var_Ctr[0] && !Var_Ctr[1]&& !Var_Ctr[3]) {
            MovSpin();
            Var_Ctr[2] = true;
        }
        if (!Var_Ctr[0] && !Var_Ctr[1] && !Var_Ctr[2]&& !Var_Ctr[3]) {
            noMov();
        }



        Var_Ctr[0] = false;
        Var_Ctr[1] = false;
        Var_Ctr[2] = false;
        Var_Ctr[3] = false;

        // if (controller.getRawButton(6)) {
        //     ResetYaw();
        // }
    }

    public void Disable_Autonomous()
    {
      Itk.Disable_Auto();
      Passo=0;
    }
    public void MovAuto1() { //CENTER deposita; coleta meio; deposita
        if (Passo==0)
        {
          Itk.setPipeline(1);
          setTEncPosition(0);
          Itk.setITKDown(true);
          Itk.setAutoProtect(true);
          MovDefault(0, 0);

          if(Itk.getFCINC()==true)
          {
            Itk.setITKDown(false);
            Itk.setAutoProtect(false);
            Passo=1;
          }
        }

        if(Passo==1)
        {
          Itk.setAutoLime(false);
          Itk.setITKUp(true);
          Auto_Cont[0]++;
          if(Auto_Cont[0]>20)
          {
            Auto_Cont[0]=0;
            Itk.setITKUp(false);
            Itk.setAutoLime(true);
            Passo=2;
          }
        }

        if(Passo==2)
        {   
            Auto_Cont[1]++;
            if(Auto_Cont[1]>=50 && Auto_Cont[1]<52)
            {
              Itk.setAutoLime(false);
            }
            if(Auto_Cont[1]>=52 && Auto_Cont[1]<53)
            {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
            }
            if(Auto_Cont[1]>=100)
            {
              Auto_Cont[1]=0;
              Passo=3;
            }
        }

        if(Passo==3)
        {
            Itk.Modo_Coletar(1);
            Itk.SetDisparo(false);
            if(Itk.getFCINC()==true)
            {
                MovDefault(0, (-18));
            }

            if(TMSD.getEncoder().getPosition() <= (-0.87)) 
            {
              Passo=4;
            }
        }

        if(Passo==4)
        {
          Itk.setITKUp(true);
          Auto_Cont[2]++;
          if(Auto_Cont[2]>70)
          {
            Auto_Cont[2]=0;
            Itk.setITKUp(true);
            Itk.setAutoLime(true);
            Passo=5;
          }
        }

        if(Passo==5)
        {
            Auto_Cont[3]++;
            if(Auto_Cont[3]>=100 && Auto_Cont[3]<102)
            {
              Itk.setITKUp(false);
              Itk.setAutoLime(false);
            }
            if(Auto_Cont[3]>=102 && Auto_Cont[3]<103)
            {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
            }
            if(Auto_Cont[3]>=150)
            {
              Auto_Cont[3]=0;
              Passo=6;
            }
        }
        if(Passo==6)
        {
          Itk.SetDisparo(false);

        }
      }

    public void MovAuto2() { //CENTER deposita; coleta meio; deposita; coleta lateral esquerda; deposita;
        if (Passo==0)
        {
          setAllIdleMode(IdleMode.kBrake);
          Itk.setAuto_Mode(true);
          Itk.setPipeline(1);
          setTEncPosition(0);
          Itk.setITKDown(true);
          Itk.setAutoProtect(true);
          MovDefault(0, 0);

          if(Itk.getFCINC()==true)
          {
            Itk.setITKDown(false);
            Itk.setAutoProtect(false);
            Passo=1;
          }
        }

        if(Passo==1)
        {
          Itk.setAutoLime(false);
          Itk.setITKUp(true);
          Auto_Cont[0]++;
          if(Auto_Cont[0]>20)
          {
            Auto_Cont[0]=0;
            Itk.setITKUp(false);
            // Itk.setAutoLime(true);
            Passo=2;
          }
        }

        if(Passo==2)
        {   
            Auto_Cont[1]++;
            // if(Auto_Cont[1]>=50 && Auto_Cont[1]<52)
            // {
            //   Itk.setAutoLime(false);
            // }
            if(Auto_Cont[1]<3)
            {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
            }
            if(Auto_Cont[1]>=90)
            {
              Auto_Cont[1]=0;
              Itk.SetDisparo(false);
              Passo=3;
            }
        }

        if(Passo==3)
        {
            double Dist_=-18;
            Itk.Modo_Coletar(1);
            if(Itk.getFCINC()==true)
            {
                MovDefault(0, (Dist_));
            }

            if(TMSD.getEncoder().getPosition() <= (Dist_+(0.5)) || Itk.getColetou()==true) 
            {
              Passo=4;
            }
        }

        if(Passo==4)
        {
          Itk.setITKUp(true);
          Auto_Cont[2]++;
          if(Auto_Cont[2]>85)
          {
            Auto_Cont[2]=0;
            Itk.setITKUp(false);
            Itk.setAutoLime(false);
            Passo=5;
          }
        }

        if(Passo==5)
        {
            Auto_Cont[3]++;
            // if(Auto_Cont[3]>=100 && Auto_Cont[3]<102)
            // {
            //   Itk.setITKUp(false);
              // Itk.setAutoLime(false);
            // }
            if(Auto_Cont[3]<2)
            {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
            }
            if(Auto_Cont[3]>=90)
            {
              Auto_Cont[3]=0;
              Itk.SetDisparo(false);
              Passo=6;
            }
        }
        if(Passo==6)
        {
          MovSpin(PID_Wrapping(getABSYaw(), 295, 0.1, 0.1, 1, 360));
          if(getABSYaw() <=(295+1) && getABSYaw() >=(295-1))
          {
            setAllTMotorsOutPut(0);
            setTEncPosition(0);
            Passo=7;
          }
        }
        if(Passo==7)
        {
             Itk.Modo_Coletar(1);
             if(Itk.getFCINC()==true)
             {
                 MovDefault(-115, 22);
             }
             else{
              MovDefault(-115, 0);
             }

             if(TMSD.getEncoder().getPosition() >= 21.5 || Itk.getColetou()==true) 
             {
              Passo=8;
             }
        }

        if(Passo==8)
        {

            MovDefault(-115, 0);

            if(TMSD.getEncoder().getPosition() <= 0.5) 
            {
             Passo=9;
            }
        }

        if(Passo == 9) 
        {
          Itk.setITKUp(true);
          
          MovSpin(PID_Wrapping(getABSYaw(), 0, 0.02, 0.1, 1, 360));
          
          Auto_Cont[4]++;
          if(Auto_Cont[4]>=85)
          {
            Itk.setITKUp(false);
          }
          
          if(Auto_Cont[4]>=100 && (getABSYaw() <=(0+2) && getABSYaw() >=(0-2)))
          {
            setAllTMotorsOutPut(0);
            Itk.setITKUp(false);
            // Itk.setAutoLime(true);
            Auto_Cont[4]=0;
            Passo=10;
          }
        }

        if(Passo==10)
        {
          // if (Itk.getTargetX() != 0) {
          //   MovSpin(Itk.PID_JP(-2, Itk.getTargetX(), 0.1, 0.1, 0.4));
          // }
          // else setAllTMotorsOutPut(0);

          Auto_Cont[5]++;
          //if(Auto_Cont[5]>=15) Itk.setPipeline(3);
          // if(Auto_Cont[5]>=90 && Auto_Cont[5]<91)
          // {
            Itk.setAutoLime(false);
          // }
          if(Auto_Cont[5]<=2)
          {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
          }
          if(Auto_Cont[5]>=90)
          {
            Auto_Cont[5]=0;
            Itk.SetDisparo(false);
            Passo=11;
            setAllTMotorsOutPut(0);
          }
        }
      }

    public void MovAuto3() { //CENTER deposita; coleta meio; deposita; coleta lateral direita; deposita;
        if (Passo==0)
        {
          setAllIdleMode(IdleMode.kBrake);
          Itk.setAuto_Mode(true);
          Itk.setPipeline(1);
          setTEncPosition(0);
          Itk.setITKDown(true);
          Itk.setAutoProtect(true);
          MovDefault(0, 0);

          if(Itk.getFCINC()==true)
          {
            Itk.setITKDown(false);
            Itk.setAutoProtect(false);
            Passo=1;
          }
        }

        if(Passo==1)
        {
          Itk.setAutoLime(false);
          Itk.setITKUp(true);
          Auto_Cont[0]++;
          if(Auto_Cont[0]>20)
          {
            Auto_Cont[0]=0;
            Itk.setITKUp(false);
            // Itk.setAutoLime(true);
            Passo=2;
          }
        }

        if(Passo==2)
        {   
            Auto_Cont[1]++;
            // if(Auto_Cont[1]>=50 && Auto_Cont[1]<52)
            // {
            //   Itk.setAutoLime(false);
            // }
            if(Auto_Cont[1]<3)
            {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
            }
            if(Auto_Cont[1]>=90)
            {
              Itk.SetDisparo(false);
              Auto_Cont[1]=0;
              Passo=3;
            }
        }

        if(Passo==3)
        {
            double Dist_=-18;
            Itk.Modo_Coletar(1);
            if(Itk.getFCINC()==true)
            {
                MovDefault(0, (Dist_));
            }

            if(TMSD.getEncoder().getPosition() <= (Dist_+(0.5)) || Itk.getColetou()==true) 
            {
              Passo=4;
            }
        }

        if(Passo==4)
        {
          Itk.setITKUp(true);
          Auto_Cont[2]++;
          if(Auto_Cont[2]>85) //70
          {
            Auto_Cont[2]=0;
            Itk.setITKUp(false);
            //Itk.setAutoLime(true);
            Itk.setAutoLime(false);

            Passo=5;
          }
        }

        if(Passo==5)
        {
            Auto_Cont[3]++;
            // if(Auto_Cont[3]>=100 && Auto_Cont[3]<102)
            // {
            //   Itk.setITKUp(false);
            //   Itk.setAutoLime(false);
            // }
            if(Auto_Cont[3]<2)
            {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
            }
            if(Auto_Cont[3]>=90)
            {
              Itk.SetDisparo(false);
              Auto_Cont[3]=0;
              Passo=6;
            }
        }
        if(Passo==6)
        {
          double Direcao = 65;
          MovSpin(PID_Wrapping(getABSYaw(), Direcao, 0.1, 0.1, 1, 360));
          if(getABSYaw() <=(Direcao+1) && getABSYaw() >=(Direcao-1))
          {
            setAllTMotorsOutPut(0);
            setTEncPosition(0);
            Passo=7;
          }
        }
        if(Passo==7)
        {
             Itk.Modo_Coletar(1);
             if(Itk.getFCINC()==true)
             {
                 MovDefault(115, 22);
             }
             else{
              MovDefault(115, 0);
             }

             if(TMSD.getEncoder().getPosition() >= 21.5 || Itk.getColetou()==true) 
             {
              Passo=8;
             }
        }
        if(Passo==8)
        {
            MovDefault(115, 0);

            if(TMSD.getEncoder().getPosition() <= 0.5) 
            {
             Passo=9;
            }
        }

        if (Passo == 9) {
          Itk.setITKUp(true);
          
          MovSpin(PID_Wrapping(getABSYaw(), 0, 0.02, 0.1, 1, 360));
          
          Auto_Cont[4]++;
          if(Auto_Cont[4]>=85)
          {
            Itk.setITKUp(false);
          }
          if(Auto_Cont[4]>=100 && (getABSYaw() <=(0+2) && getABSYaw() >=(0-2)))
          {
            setAllTMotorsOutPut(0);
            Itk.setITKUp(false);
            // Itk.setAutoLime(true);
            Auto_Cont[4]=0;
            Passo=10;
          }
        }

        if(Passo==10)
        {
          // if (Itk.getTargetX() != 0) {
          //   MovSpin(Itk.PID_JP(-2, Itk.getTargetX(), 0.2, 0.1, 0.4));
          // }
          // else setAllTMotorsOutPut(0);

          Auto_Cont[5]++;
          // if(Auto_Cont[5]>=90 && Auto_Cont[5]<91)
          // {
          //   Itk.setAutoLime(false);
          // }
          if(Auto_Cont[5]<=2)
          {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
          }
          if(Auto_Cont[5]>=90)
          {
            Auto_Cont[5]=0;
            Itk.SetDisparo(false);
            Passo=11;
            setAllTMotorsOutPut(0);
          }
        }
      }

    public void MovAuto4() { //RIGHT deposita; coleta lateral direita; deposita;
        if (Passo==0)
        {
          setAllIdleMode(IdleMode.kBrake);
          Itk.setAuto_Mode(true);
          //Itk.setPipeline(1);
          setTEncPosition(0);
          Itk.setITKDown(true);
          Itk.setAutoProtect(true);
          MovDefault(0, 0);

          if(Itk.getFCINC()==true)
          {
            Itk.setITKDown(false);
            Itk.setAutoProtect(false);
            Passo=1;
          }
        }
        if(Passo==1)
        {
          Itk.setITKUp(true);

          MovSpin(PID_Wrapping(getABSYaw(), 50, 0.02, 0.1, 1, 360));
          
          Auto_Cont[0]++;
          if(Auto_Cont[0]>=85)
          {
            Itk.setITKUp(false);
          }
          if(Auto_Cont[0]>=85 && (getABSYaw() <=(50+2) && getABSYaw() >=(50-2)))
          {
            setAllTMotorsOutPut(0);
            Itk.setITKUp(false);
            // Itk.setAutoLime(true);
            Auto_Cont[0]=0;
            Passo=2;
          }
        }

        if(Passo==2)
        {

          // if (Itk.getTargetX() != 0) {
          //   MovSpin(Itk.PID_JP(-2, Itk.getTargetX(), 0.5, 0.1, 0.3));
          // }
          // else setAllTMotorsOutPut(0);

          Auto_Cont[2]++;
          // if(Auto_Cont[2]>=90 && Auto_Cont[2]<91)
          // {
          //   Itk.setAutoLime(false);
          // }
          if(Auto_Cont[2]<=2)
          {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
          }
          if(Auto_Cont[2]>=90)
          {
            Auto_Cont[2]=0;
            Itk.SetDisparo(false);
            Passo=3;
            setAllTMotorsOutPut(0);
          }
        }
        if(Passo==3)
        {
          MovSpin(PID_Wrapping(getABSYaw(), 0, 0.02, 0.1, 1, 360));
          
          if((getABSYaw() <=(1) && getABSYaw() >=(-1)))
          {
            setAllTMotorsOutPut(0);
            Itk.setITKUp(false);
            Itk.setAutoLime(false);
            MovDefault(0, 0);
            setTEncPosition(0);
            Passo=4;
          }
        }
        if(Passo==4)
        {

            double Dist_=-18;
            Itk.Modo_Coletar(1);
            Itk.SetDisparo(false);
            if(Itk.getFCINC()==true)
            {
                MovDefault(0, (Dist_));
            }

            if(TMSD.getEncoder().getPosition() <= (Dist_+(0.5))) 
            {
              Passo=5;
            }
        }
        if(Passo==5)
        {
          Itk.setITKUp(true);

          MovSpin(PID_Wrapping(getABSYaw(), 35, 0.02, 0.1, 1, 360));
          
          Auto_Cont[4]++;
          if(Auto_Cont[4]>=130)
          {
            Itk.setITKUp(false);
          }
          if(Auto_Cont[4]>=130 && (getABSYaw() <=(35+2) && getABSYaw() >=(35-2)))
          {
            setAllTMotorsOutPut(0);
            Itk.setITKUp(false);
            // Itk.setAutoLime(true);
            Auto_Cont[4]=0;
            Passo=6;
          }
        }
        if(Passo==6)
        {
          Auto_Cont[7]++;

          // if (Itk.getTargetX() != 0) {
          //   MovSpin(Itk.PID_JP(-2, Itk.getTargetX(), 0.5, 0.1, 0.3));
          // }
          // else setAllTMotorsOutPut(0);

          // if(Auto_Cont[7]>=90 && Auto_Cont[7]<92)
          // {
          //   Itk.setAutoLime(false);
          // }
          if(Auto_Cont[7]<=2)
          {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
          }
          if(Auto_Cont[7]>=90)
          {
            Auto_Cont[7]=0;
            Itk.SetDisparo(false);
            setAllTMotorsOutPut(0);
            Passo=7;
            setAllTMotorsOutPut(0);
          }
        }


      }
    public void MovAuto5() { //LEFT deposita; coleta lateral esquerda; deposita;
        if (Passo==0)
        {
          setAllIdleMode(IdleMode.kBrake);
          Itk.setAuto_Mode(true);
          Itk.setPipeline(1);
          setTEncPosition(0);
          Itk.setITKDown(true);
          Itk.setAutoProtect(true);
          MovDefault(0, 0);

          if(Itk.getFCINC()==true)
          {
            Itk.setITKDown(false);
            Itk.setAutoProtect(false);
            Passo=1;
          }
        }
        if(Passo==1)
        {
          Itk.setITKUp(true);

          MovSpin(PID_Wrapping(getABSYaw(), 310, 0.04, 0.1, 1, 360));

          Auto_Cont[0]++;

          if(Auto_Cont[0]>=85)
          {
            Itk.setITKUp(false);
          }
          if(Auto_Cont[0]>=85  && (getABSYaw() <=(310+2) && getABSYaw() >=(310-2)))
          {
            setAllTMotorsOutPut(0);
            Itk.setITKUp(false);
            Auto_Cont[0]=0;
            Passo=2;
          }
        }

        if(Passo==2)
        {

          // if (Itk.getTargetX() != 0) {
          //   MovSpin(Itk.PID_JP(-2, Itk.getTargetX(), 0.1, 0.1, 0.4));
          // }
          // else setAllTMotorsOutPut(0);
          // if((Itk.getTargetX()<=(-1)) && (Itk.getTargetX()>=(-3))){
          Auto_Cont[2]++;
          //}
          // if(Auto_Cont[2]>=90 && Auto_Cont[2]<91)
          // {
          //   Itk.setAutoLime(false);
          // }
          if(Auto_Cont[2]<=2)
          {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
          }
          if(Auto_Cont[2]>=90)
          {
            Auto_Cont[2]=0;
            Itk.SetDisparo(false);
            Passo=3;
            setAllTMotorsOutPut(0);
          }

        }
        if(Passo==3)
        {
          MovSpin(PID_Wrapping(getABSYaw(), 0, 0.02, 0.1, 1, 360));
          
          if((getABSYaw() <=(1) && getABSYaw() >=(-1)))
          {
            setAllTMotorsOutPut(0);
            Itk.setITKUp(false);
            Itk.setAutoLime(false);
            MovDefault(0, 0);
            setTEncPosition(0);
            Passo=4;
          }
        }
        if(Passo==4)
        {
            double Dist_=-18;
            Itk.Modo_Coletar(1);
            Itk.SetDisparo(false);
            if(Itk.getFCINC()==true)
            {
                MovDefault(0, (Dist_));
            }

            if(TMSD.getEncoder().getPosition() <= (Dist_+(0.5))) 
            {
              Passo=5;
            }
        }
        if(Passo==5)
        {
          Itk.setITKUp(true);

          MovSpin(PID_Wrapping(getABSYaw(), 325, 0.02, 0.1, 1, 360));
          
          Auto_Cont[4]++;
            if(Auto_Cont[0]>=130)
          {
            Itk.setITKUp(false);
          }
          
          if(Auto_Cont[4]>=130 && (getABSYaw() <=(325+2) && getABSYaw() >=(325-2)))///
          {
            setAllTMotorsOutPut(0);
            Itk.setITKUp(false);
            // Itk.setAutoLime(true);
            Auto_Cont[4]=0;
            Passo=6;
          }
        }
        if(Passo==6)
        {
          Auto_Cont[7]++;

          // if (Itk.getTargetX() != 0) {
          //   MovSpin(Itk.PID_JP(-2, Itk.getTargetX(), 0.5, 0.1, 0.4));
          // }
          // else setAllTMotorsOutPut(0);

          if(Auto_Cont[7]>=90 && Auto_Cont[7]<92)
          {
            Itk.setAutoLime(false);
          }
          if(Auto_Cont[7]>=92 && Auto_Cont[7]<=94)
          {
              Itk.AutoSetITK();
              Itk.SetDisparo(true);
          }
          if(Auto_Cont[7]>=160)
          {
            Auto_Cont[7]=0;
            Itk.SetDisparo(false);
            setAllTMotorsOutPut(0);
            Passo=7;
            setAllTMotorsOutPut(0);
          }
        }
      }

    @Override
    public void periodic() 
    {
        Cont[0]++;
        SmartDashboard.putNumber("teste", Cont[0]);
        if (Cont[0] > 100) {
            setCCABS4MotorEnc();
            Cont[0] = 0;
        }
        Itk.periodic();
    }
}



