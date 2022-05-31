package org.firstinspires.ftc.teamcode.opMode.protoType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Acquirer;
@TeleOp
public class droptest extends LinearOpMode {
    Acquirer drop = new Acquirer();
    @Override
    public void runOpMode() throws InterruptedException {
        drop.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                drop.flipUp();
            }
            if(gamepad1.b) {
                drop.flipDown();
            }
            if(gamepad1.x) {
                drop.intake(0.5);
            }else {
                drop.intake(0);
            }
        }
    }
}
