package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class SCORINGFSMTurretMode extends Mechanism {
    public LIFTFSM lift = new LIFTFSM();
    public ARMFSM arm = new ARMFSM();
    public Turret turret = new Turret();
    ElapsedTime timer = new ElapsedTime();
    FreightSensor sensor = new FreightSensor();
    public static double turretMove = 1250;
    public enum states {
        down,
        readyH,
        readyM,
        readyL,
        score,
        downPrep,
        readyArm
    }
    public states scoreStates;
    @Override
    public void init(HardwareMap hwMap) {
        scoreStates = states.down;
        lift.init(hwMap);
        arm.init(hwMap);
        turret.init(hwMap);
        sensor.init(hwMap);
    }
    public void init(HardwareMap hwMap, Telemetry tele) {
        lift.init(hwMap, tele);
        arm.init(hwMap, tele);
        sensor.init(hwMap);
    }
    public void loop() {
        switch(scoreStates) {
            case down:
                if(timer.milliseconds() >= 450) {
                    lift.goLow();
                    arm.down();
                }
                break;
            case readyH:
                timer.reset();
                lift.goHigh();
                arm.ready();
                scoreStates = states.readyArm;
                break;
            case readyM:
                timer.reset();
                lift.goMid();
                arm.ready();
                scoreStates = states.readyArm;
                break;
            case readyL:
                timer.reset();
                lift.goLow();
                arm.ready();
                scoreStates = states.readyArm;
                break;
            case score:
                timer.reset();
                arm.dump();
                scoreStates = states.downPrep;
                break;
            case downPrep:
                if(timer.milliseconds() >= 450){
                    timer.reset();
                    turret.middle();
                    scoreStates = states.down;
                }
                break;

            case readyArm:
                if(timer.milliseconds() >= turretMove) {
                    turret.defaultSide();
                }
                break;
        }
        lift.loop();
        arm.loop();
    }
    public void highGoal() {
        scoreStates = states.readyH;
    }
    public void midGoal() {
        scoreStates = states.readyM;
    }
    public void lowGoal() {
        scoreStates = states.readyL;
    }
    public void score() {
        scoreStates = states.score;
    }
    public void down() {scoreStates = states.downPrep;}
    public void toggleHigh(){
        if(scoreStates != states.readyArm) {
            highGoal();
        }else{
            down();
        }
    }
    public void toggleMid(){
        if(scoreStates != states.readyArm) {
            midGoal();
        }else{
            down();
        }
    }
    public void toggleLow(){
        if(scoreStates != states.readyArm) {
            lowGoal();
        }else{
            down();
        }
    }

    public void setDefaultSide(Turret.Side side){

        turret.setDefaultSide(side);
    }

    public void setLeftDefault(){
        setDefaultSide(Turret.Side.LEFT);
    }

    public void setRightDefault(){
        setDefaultSide(Turret.Side.RIGHT);
    }

    public void setMiddleDefault(){
        setDefaultSide(Turret.Side.MIDDLE);
    }
}
