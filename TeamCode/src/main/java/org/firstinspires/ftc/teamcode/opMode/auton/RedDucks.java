package org.firstinspires.ftc.teamcode.opMode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.CapVision;
import org.firstinspires.ftc.teamcode.hardware.Capper;
import org.firstinspires.ftc.teamcode.hardware.Carousel;
import org.firstinspires.ftc.teamcode.hardware.RetractableOdoSys;
import org.firstinspires.ftc.teamcode.hardware.TURRETFSM;
import org.firstinspires.ftc.teamcode.hardware.util.DelayCommand;
import org.firstinspires.ftc.teamcode.hardware.FreightSensor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous (group = "RedAuton")
public class RedDucks extends LinearOpMode {
    private CapVision cv = new CapVision();
    private Carousel carousel = new Carousel();
    private DelayCommand delay = new DelayCommand();
    private FreightSensor sensor = new FreightSensor();
    private TURRETFSM scoringMech= new TURRETFSM();
    private RetractableOdoSys odoSys = new RetractableOdoSys();
    private Capper cap = new Capper();

    public int theGoal;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    public static double startx = -36.0;
    public static double starty = 70.0;
    public static double startAng = Math.toRadians(90);

    public static double scoreHubPosx = -34;
    public static double scoreHubPosy = 29;

    public static double scoreHubPosAngB = -25;
    public static double scoreHubPosAngR = 45;

    public static double carouselPosx = -60.5;
    public static double carouselPosy = 63.5;
    public static double carouselPosAng = Math.toRadians(270);

    public static double parkX = -63;
    public static double parkY = 42;
    public static double parkAng = Math.toRadians(180);

    public static String goal = "highgoal";

    Pose2d startPosR = new Pose2d(startx, -starty, -startAng);
    Vector2d scoreHubPosR = new Vector2d(scoreHubPosx, -scoreHubPosy);
    Pose2d carouselPosR = new Pose2d(carouselPosx, -carouselPosy, carouselPosAng);
    Pose2d parkR = new Pose2d(parkX, -parkY, parkAng);


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //initialize mechanisms
        carousel.init(hardwareMap);
        sensor.init(hardwareMap);
        cap.init(hardwareMap);

        scoringMech.init(hardwareMap);
        cv.init(hardwareMap);
        odoSys.init(hardwareMap, true);


        //drive train + async updates of mechanisms
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setSlides(scoringMech);

        //important coordinates here

        //set startPose
        drive.setPoseEstimate(startPosR);

        //trajectory


        //3ftx3ftmovement

//        TrajectorySequenceBuilder taahkbeer = drive.trajectorySequenceBuilder(alFatihah.build().end())
//                .splineTo(new Vector2d(bankcurveX,bankcurveY),Math.toRadians(90))
//                .addDisplacementMarker(()->{
//                    drive.acquirerRuns = true;
//                })
//                .forward(tuningNumber)
//                .waitSeconds(2);

//        TrajectorySequenceBuilder allahhuackbar = drive.trajectorySequenceBuilder(taahkbeer.build().end())
//                .back(tuningNumber)
//                .addDisplacementMarker(() -> {
//                    scoringMech.toggle("highgoal");
//                    drive.acquirerRuns = false;
//                })
//                .setReversed(true)
//                .splineTo(new Vector2d(18, starty), Math.toRadians(0))
//                .UNSTABLE_addTemporalMarkerOffset(0,()->{
//                    scoringMech.release();
//                });


        //mashallah

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Waiting in init");
            telemetry.update();
        }
        theGoal = cv.whichRegion();
        telemetry.addData("goal: ",goal);
        telemetry.addData("region", cv.whichRegion());
        telemetry.update();
        TrajectorySequence duckyPath = drive.trajectorySequenceBuilder(startPosR)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(parkX, -scoreHubPosy), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.toggleGoal(theGoal);
                })
                .lineToLinearHeading(new Pose2d(scoreHubPosR, Math.toRadians(180)))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scoringMech.score();
                })
                .waitSeconds(1)
                // slides
                .setReversed(false)
                .splineTo(new Vector2d(parkX, -parkY), Math.toRadians(270))
                .lineToSplineHeading(carouselPosR)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    carousel.run(false,true);
                })
                .build();
        TrajectorySequence parka = drive.trajectorySequenceBuilder(carouselPosR)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    carousel.run(false,false);
                })
                .lineToSplineHeading(parkR)

                .build();

//
//        scoringMech.toggle(goal);
        drive.followTrajectorySequence(duckyPath);
        ElapsedTime timer = new ElapsedTime();
        while(timer.seconds() <= 3) {
            if(timer.seconds() <= 1.5) {
                carousel.rrrun(timer, -1);
            }else {
                carousel.runmax(false, true);
            }
        }
        drive.followTrajectorySequence(parka);
    }
}