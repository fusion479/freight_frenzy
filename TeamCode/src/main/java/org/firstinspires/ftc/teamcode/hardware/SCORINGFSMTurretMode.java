package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SCORINGFSMTurretMode extends Mechanism {
    public LIFTFSM lift = new LIFTFSM();
    public ARMFSM arm = new ARMFSM();
    public Turret turret = new Turret();
    ElapsedTime timer = new ElapsedTime();
    FreightSensor sensor = new FreightSensor();
    public enum states {
        down,
        readyH,
        readyM,
        readyL,
        score
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
                turret.middle();
                break;
            case readyH:
                lift.goHigh();
                arm.ready();
                turret.toggleDefault();
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
}
