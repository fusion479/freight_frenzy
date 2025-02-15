package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class SCORINGFSM extends Mechanism {
    public LIFTFSM lift = new LIFTFSM();
    public ARMFSM arm = new ARMFSM();
    ElapsedTime timer = new ElapsedTime();
    FreightSensor sensor = new FreightSensor();
    public static int liftTimer = 450;
    public enum states {
        down,
        readyH,
        readyM,
        readyL,
        readyS,
        score
    }
    public states scoreStates;
    @Override
    public void init(HardwareMap hwMap) {
        scoreStates = states.down;
        lift.init(hwMap);
        arm.init(hwMap);
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
                if(timer.milliseconds() >= liftTimer) {
                    lift.goLow();
                    if (timer.milliseconds() >= liftTimer + 100){
                        arm.down();
                    }
                }
                break;
            case readyS:
                lift.share();
                arm.ready();
                break;
            case readyH:
                lift.goHigh();
                arm.ready();
                break;
            case readyM:
                lift.goMid();
                arm.ready();
                break;
            case readyL:
                lift.goLow();
                arm.ready();
                break;
            case score:
                timer.reset();
                arm.dump();
                scoreStates = states.down;
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
    public void shareGoal() {scoreStates = states.readyS;}
    public void down() {scoreStates = states.down;}
    public void toggleHigh(){
        if(scoreStates != states.readyH) {
            highGoal();
        }else{
            down();
        }
    }
    public void toggleMid(){
        if(scoreStates != states.readyM) {
            midGoal();
        }else{
            down();
        }
    }
    public void toggleLow(){
        if(scoreStates != states.readyL) {
            lowGoal();
        }else{
            down();
        }
    }
    public void setGoal(int goal) {
        if(goal == 3) {
            highGoal();
        }
        if(goal == 2) {
            midGoal();
        }
        if(goal == 1) {
            lowGoal();
        }
        if(goal == 4) {
            shareGoal();
        }
    }
}
