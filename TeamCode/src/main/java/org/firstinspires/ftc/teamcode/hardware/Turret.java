package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Turret extends Mechanism{

    public enum Side {
        LEFT,
        RIGHT,
        MIDDLE
    }


    public Side side = Side.MIDDLE;
    public Side defaultSide = Side.MIDDLE;
    public static double armEnd = 1.0;
    public static double armStart = 0.2;

    public static double middle = 0.32;
    ServoManager turret = new ServoManager("turret",armStart,armEnd);
    public void init(HardwareMap hwmap){
        turret.init(hwmap);
        middle();
    }

    public void resetLimits(){
        turret.resetLimits(armStart,armEnd);
    }

    public void setPosRatio(double pos){
        turret.setPosRatio(pos);
    }


    public void left(){
        turret.setPosRatio(0);
        side = Side.LEFT;
    }

    public void right(){
        turret.setPosRatio(1);
        side = Side.RIGHT;

    }

    public void middle(){
        turret.setPosRatio(middle);
        side = Side.MIDDLE;
    }

    public void setDefaultSide(Side side){
        defaultSide = side;
    }

    public void defaultSide(){
        if(defaultSide == Side.LEFT){
            left();
        }
        if(defaultSide == Side.RIGHT){
            right();
        }
        if(defaultSide == Side.MIDDLE) {
            middle();
        }
    }

    public void toggleDefault(){
        if(side != Side.MIDDLE){
            middle();
        }

        else{
            defaultSide();
        }
    }

    public void setPosAngle(double ang){
        double angConversion = ang / 180.0;

    }

    public void analogToPos(double X, double Y){

    }


}
