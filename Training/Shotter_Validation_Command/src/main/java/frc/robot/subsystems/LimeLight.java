package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class LimeLight {
    private static NetworkTable limeTable;
    private static int pipelineSelect = 1;
    private static int ledMode = 1;

    private static final String pipe0  = "0";
    private static final String pipe1  = "1";
    private static final String pipe2  = "2";
    private static final String pipe3  = "3";
    private static final String pipe4  = "4";
    private static final String pipe5  = "5";
    private static final String pipe6  = "6";
    private String m_PipeSelect;
    private final SendableChooser<String> m_Pipe = new SendableChooser<>();

    private static final String limeLED0  = "0";
    private static final String limeLED1  = "1";
    private static final String limeLED2  = "2";
    private static final String limeLED3  = "3";

    private String m_LimeLEDSelect;
    private final SendableChooser<String> m_LimeLED = new SendableChooser<>();

    public static double targeTX = 0;
    public static double targeTY = 0;
    public static double targeTArea = 0;


    public LimeLight() {
        m_LimeLED.setDefaultOption("MODE: Pipeline", limeLED0); 
        m_LimeLED.addOption("MODE: Off", limeLED1);       
        m_LimeLED.addOption("MODE: Piscar", limeLED2);       
        m_LimeLED.addOption("MODE: Force On", limeLED3); 
        SmartDashboard.putData("LIMELED MODE:", m_LimeLED);
        SmartDashboard.putData(CommandScheduler.getInstance());
        
        m_Pipe.setDefaultOption("Blue 1x", pipe0);      
        m_Pipe.addOption("Blue 1x", pipe1);
        m_Pipe.addOption("Blue 2x", pipe2);
        m_Pipe.addOption("Blue 3x", pipe3);
        m_Pipe.addOption("Red 1x", pipe4);
        m_Pipe.addOption("Red 2x", pipe5);
        m_Pipe.addOption("Red 3x", pipe6);
        SmartDashboard.putData("PIPELINE SELECT:", m_Pipe);
        SmartDashboard.putData(CommandScheduler.getInstance());
        limeTable = NetworkTableInstance.getDefault().getTable("limelight");

        //LimeTableFront = NetworkTableInstance.getDefault().getTable("limelight-frente");
    }

  public void debugLime() {
    SmartDashboard.putNumber("TargetTx", targeTX);
    SmartDashboard.putNumber("TargetTy", targeTY);
    SmartDashboard.putNumber("Pipeline", pipelineSelect);
  }


  public void periodic() {
    m_LimeLEDSelect = m_LimeLED.getSelected();
    switch (m_LimeLEDSelect) {
      case "0":
        ledMode=0;
        break;
      case "1":
        ledMode=1;
        break;
      case "2":
        ledMode=2;
        break;
      case "3":
        ledMode=3;
        break;
      default:
        ledMode=1;
        break;
    }
    m_PipeSelect = m_Pipe.getSelected();
    switch (m_PipeSelect) {
      case "0":
        pipelineSelect=0;
        break;
      case "1":
        pipelineSelect=1;
        break;
      case "2":
        pipelineSelect=2;
        break;
      case "3":
        pipelineSelect=3;
        break;
      case "4":
        pipelineSelect=4;
        break;
      case "5":
        pipelineSelect=5;
        break;
      case "6":
        pipelineSelect=6;
        break;
      default:
        pipelineSelect=1;
        break;
    }

    limeTable.getEntry("pipeline").setNumber(pipelineSelect);  // Define o valor da pipeline desejada
    limeTable.getEntry("ledMode").setNumber(ledMode);          // Modo do Led definido para 1

    targeTX = limeTable.getEntry("tx").getDouble(0.0);
    targeTY = limeTable.getEntry("ty").getDouble(0.0);
    targeTArea = limeTable.getEntry("ta").getDouble(0.0);
    // if (limeTable.getEntry("tx").getDouble(0.0)!=0 && limeTable.getEntry("ty").getDouble(0.0)!=0) {
    // //   TargetTx_Anterior = limeTable.getEntry("tx").getDouble(0.0);
    // //   TargetTy_Anterior = limeTable.getEntry("ty").getDouble(0.0);
    // }
    debugLime();
  }

  public void limeTeleopPeriodic() {
    
  }
    
}
