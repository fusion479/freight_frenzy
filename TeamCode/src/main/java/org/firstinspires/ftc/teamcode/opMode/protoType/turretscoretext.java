package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.TURRETFSM;

@TeleOp
public class turretscoretext extends LinearOpMode {
    TURRETFSM truman = new TURRETFSM();
    @Override
    public void runOpMode() throws InterruptedException {
        truman.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                truman.high();
            }
            if (gamepad1.b) {
                truman.low();
            }
            if (gamepad1.x) {
                truman.mid();
            }
            if (gamepad1.y) {
                truman.score();
            }
            if (gamepad1.left_stick_button) {
                truman.down();
            }
            truman.loop();
            telemetry.addData("hmm", truman.score.scoreStates);
            telemetry.addData("turr", truman.s);
        }
    }
}
