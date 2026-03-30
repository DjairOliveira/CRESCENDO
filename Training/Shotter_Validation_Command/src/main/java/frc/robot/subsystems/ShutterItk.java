package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shutter.Disparar;
import frc.robot.Constants.ConstShutter;

public class ShutterItk {
    int inProgres;
    double vMotorDisp;
    double vMotorItk;
    double tAvanco;
    double tRecuo;

    public ShutterItk(double timerAvanco, double timerRecuo){
        inProgres=0;
        vMotorDisp=-0.2;
        vMotorItk=1;

        tAvanco=timerAvanco;
        tRecuo=timerRecuo;

        ConstShutter.shutter_disp_sup.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_disp_inf.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_itk.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_disp_sup.setInverted(false);
    }
        public class GetNote extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                /*
                 * Posicionar o cabeçote na posição certa
                 * para pegar a nota
                 */
                
                if(ConstShutter.sensor_shutterItk.get()==true && inProgres==0 && RobotContainer.engatilhado==false)
                {
                  inProgres=1;
                }

                if(inProgres==1)
                {
                    ConstShutter.shutter_itk.set(vMotorItk*(-1));
                    if(ConstShutter.sensor_shutterItk.get()==false)
                    {
                      inProgres=2;
                    }
                }
        
                if(inProgres==2) // Avança a note
                {
                    ConstShutter.shutter_itk.set(vMotorItk*(-1));
                    Timer.delay(tAvanco);
                    inProgres=3;
                }
                if(inProgres==3) // Recuo
                {
                    ConstShutter.shutter_itk.set(vMotorItk);
                    ConstShutter.shutter_disp_inf.set(vMotorDisp);
                    ConstShutter.shutter_disp_sup.set(vMotorDisp);
                    Timer.delay(tRecuo);
                    inProgres=4;

                }
                if(inProgres==4)
                {
                    ConstShutter.shutter_disp_inf.set(0);
                    ConstShutter.shutter_disp_sup.set(0);
                    ConstShutter.shutter_itk.set(0);
                    RobotContainer.engatilhado=true;
                    inProgres=0;
                    interrupt();
                  
                }
            }
            try{
                Thread.sleep(100);
            } catch(InterruptedException e){
                interrupt();
                return;
            }
        }
    }
    public void getNote()
    {
        GetNote pegar = new GetNote();
        pegar.start();
    }

}
