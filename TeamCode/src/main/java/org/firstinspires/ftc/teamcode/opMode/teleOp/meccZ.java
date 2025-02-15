package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.MeccRobotProtoZ;
@TeleOp
public class meccZ extends LinearOpMode {
    private MeccRobotProtoZ robot = new MeccRobotProtoZ();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot.init(hardwareMap,telemetry,timer);


        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Waiting in init");
            telemetry.addData("turret side", robot.scoring.turret.side);
            telemetry.addData("scoring state", robot.scoring.s);
            telemetry.update();
        }
        while(opModeIsActive()){
            robot.run(gamepad1,gamepad2);
            //These are the RATIO positions of the servos
//            telemetry.addData("kF: ",lift.kF);
//            telemetry.addData("kP: ", lift.coeffs.kP);
//              telemetry.addData("liftL: ", .liftLeft.getCurrentPosition());
//              telemetry.addData("liftR: ", lift.liftRight.getCurrentPosition());
            telemetry.addData("turret side", robot.scoring.turret.side);
            telemetry.addData("scoring state", robot.scoring.s);
            telemetry.update();
        }
    }

}