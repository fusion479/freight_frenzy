package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.TURRETFSM;
import org.firstinspires.ftc.teamcode.hardware.util.BooleanManager;

@TeleOp
public class ScoringTester extends LinearOpMode {
    private TURRETFSM scoring = new TURRETFSM();
    BooleanManager abutton = new BooleanManager(() -> {
       scoring.toggleHigh();
    });
    BooleanManager bbutton = new BooleanManager(() -> {
        scoring.toggleMid();
    });
    BooleanManager xbutton = new BooleanManager(() -> {
        scoring.toggleLow();
    });
    BooleanManager ybutton = new BooleanManager(() -> {
        scoring.score();
    });
    BooleanManager padleft = new BooleanManager(() -> {
        scoring.setDirLeft();
    });
    BooleanManager padright = new BooleanManager(() -> {
        scoring.setDirRight();
    });
    @Override
    public void runOpMode() {
        scoring.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.clearAll();
            telemetry.addLine("A: High Goal");
            telemetry.addLine("B: Mid Goal");
            telemetry.addLine("X: Low Goal");
            telemetry.addLine("Y: Score");
            telemetry.addLine("DPad Left: Turret left");
            telemetry.addLine("DPad Right: Turret right");
            telemetry.update();
            abutton.update(gamepad1.a);
            bbutton.update(gamepad1.b);
            xbutton.update(gamepad1.x);
            ybutton.update(gamepad1.y);
            padleft.update(gamepad1.dpad_left);
            padright.update(gamepad1.dpad_right);
            scoring.loop();
        }
    }
}
