package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ConstShutter;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shutter.Disparar;

public class Turret {
    private double minPosiInc;
    private double maxPosiInc;
    private int inProgres;
    private boolean inRange;
    private double movePosi;
    private boolean shutterDown;
    private boolean setMode;
    private double setP;

    public Turret(double minPositionInc, double maxPositionInc, double setPShutter, double setOutRangeMinShutter, double setOutRangeMaxShutter)
    {
        minPosiInc=minPositionInc;
        maxPosiInc=maxPositionInc;
        inProgres=0;
        inRange=false;
        shutterDown=false;
        setMode=false;
        setP=setPShutter;

        ConstShutter.pidShutter.setP(setP);  // 0.42
        ConstShutter.pidShutter.setI(0);
        ConstShutter.pidShutter.setD(0);
        ConstShutter.pidShutter.setIZone(0);
        ConstShutter.pidShutter.setFF(0);
        ConstShutter.pidShutter.setOutputRange(setOutRangeMinShutter, setOutRangeMaxShutter);  // -0.1 e 0.25

        ConstShutter.shutter_inc_left.setOpenLoopRampRate(1);
        ConstShutter.shutter_inc_right.setOpenLoopRampRate(1);
        ConstShutter.shutter_inc_left.follow(ConstShutter.shutter_inc_right, true);

        ConstShutter.shutter_inc_right.setIdleMode(IdleMode.kBrake);
        ConstShutter.shutter_inc_left.setIdleMode(IdleMode.kBrake);

        ConstShutter.shutter_inc_right.getEncoder().setPosition(0);
        ConstShutter.shutter_inc_left.getEncoder().setPosition(0);
    }

    public class MoveShotter extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                
                if(movePosi>=minPosiInc && movePosi<=maxPosiInc){
                    inRange=true;
                }
                else{
                    inRange=false;
                    interrupt();
                }

                if(inRange==true){
                    if(inProgres==0 && movePosi>=1){
                        ConstShutter.pidShutter.setP(setP);
                        ConstShutter.pidShutter.setReference(movePosi, ControlType.kPosition);
                        interrupt();
                    }
                    if(movePosi<1 && shutterDown==false){
                        ConstShutter.pidShutter.setP(setP);
                        ConstShutter.pidShutter.setReference(movePosi, ControlType.kPosition);
                        shutterDown=true;
                    }

                    if(ConstShutter.shutter_inc_right.getEncoder().getPosition()<1 && shutterDown==true){
                        ConstShutter.pidShutter.setP(0);
                        shutterDown=false;
                        interrupt();
                    }
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
    public void periodic(){
        if(movePosi>=minPosiInc && movePosi<=maxPosiInc && LimeLight.targeTArea!=0){
            inRange=true;
        }
        else{
            inRange=false;
        }

        if(inRange==true){
            if(setMode==true){
                if(inProgres==0 && movePosi>=1){
                    ConstShutter.pidShutter.setP(setP);
                    ConstShutter.pidShutter.setReference(movePosi, ControlType.kPosition);
                }
                if(movePosi<1 && shutterDown==false){
                    ConstShutter.pidShutter.setP(setP);
                    ConstShutter.pidShutter.setReference(movePosi, ControlType.kPosition);
                    shutterDown=true;
                }
                if(ConstShutter.shutter_inc_right.getEncoder().getPosition()<1 && shutterDown==true){
                    ConstShutter.pidShutter.setP(0);
                    shutterDown=false;
                }
            }
        }
    }

    public void moveShotter(double movePosition)
    {
        movePosi=movePosition;
        setMode=false;
        MoveShotter moveSht = new MoveShotter();
        moveSht.start();
    }
    public void moveShotterTarget(double movePosition)
    {
        movePosi=movePosition;
        setMode=true;
    }
}
