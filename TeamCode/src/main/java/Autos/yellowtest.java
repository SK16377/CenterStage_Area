package Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Hardware.Arm;
import Hardware.Intake;
import Hardware.Lift;
import Hardware.yellowPixelBoxPipeline;

//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;


@Autonomous(name="yellowtest", group="Auto")
public class yellowtest extends LinearOpMode {
    SampleMecanumDrive drive;
    OpenCvCamera webcam;
    Lift lift;
    Arm arm;

    Intake intake;
    //Arm arm;
    double FRONTRELEASE = .472;
    double REARRELEASE = .235;
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
       yellowPixelBoxPipeline detector = new yellowPixelBoxPipeline(telemetry);
        //START_POSITION position = new CenterstageDetector(telemetry);
        webcam.setPipeline(detector);
        //private detector.getLocation location = detector.getLocation().LEFT;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                                         @Override
                                         public void onOpened() {
                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                         }

                                         @Override
                                         public void onError(int errorCode) {
                                             //This will be called if the camera could not be opened
                                         }
                                     }

        );
        arm.intakePos();

        Pose2d startPose = new Pose2d(-40, -63.42, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        // Pose2d rightintakepos = new Pose2d(-40, -63.42, Math.toRadians(-165));


        Trajectory left = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-45, -43.81), Math.toRadians(90))
                .build();

        Trajectory middle = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-37.5, -36.5), Math.toRadians(90))
                .build();


        Trajectory right = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-31.5, -39.5), Math.toRadians(50.00))
                .build();



        Trajectory backup_left = drive.trajectoryBuilder(left.end())
                .back(3)
                .build();
        Trajectory turn_leftaround = drive.trajectoryBuilder(backup_left.end())
                .strafeRight(1)
                .build();

        Trajectory left_stage = drive.trajectoryBuilder(turn_leftaround.end())
                .splineTo(new Vector2d(-24.76, -15.86), Math.toRadians(0.00))
                .build();
        Trajectory left_straight = drive.trajectoryBuilder(left_stage.end())
                .forward(47)
                .build();

        Trajectory left_drop = drive.trajectoryBuilder(left_straight.end())
                .splineTo(new Vector2d(41.24, -34), Math.toRadians(0.00))
                .build();
        Trajectory deposit_left = drive.trajectoryBuilder(left_drop.end())
                .forward(5.9)
                .build();
        Trajectory away_left = drive.trajectoryBuilder(deposit_left.end())
                .back(4.5)
                .build();
        Trajectory left_park = drive.trajectoryBuilder(away_left.end())
                .strafeLeft(15)
                .build();

        Trajectory backup_right = drive.trajectoryBuilder(right.end())
                .back(13)
                .build();


        Trajectory right_driveintake = drive.trajectoryBuilder(backup_right.end())
                .lineToLinearHeading(new Pose2d(-55, -36.1, Math.toRadians(0.00)))
                .build();
        Trajectory right_straight = drive.trajectoryBuilder(right_driveintake.end())
                .back(4.055)
                .build();
        Trajectory right_intakee = drive.trajectoryBuilder(right_driveintake.end())
                .back(3)
                .build();
        Trajectory right_truss = drive.trajectoryBuilder(right_straight.end())
                .lineToLinearHeading(new Pose2d(-42.5, -50, Math.toRadians(0.00)))
                .build();
        Trajectory right_strafe_ = drive.trajectoryBuilder(right_truss.end())
                .strafeRight(8.35)
                .build();

        Trajectory right_under_truss = drive.trajectoryBuilder(right_strafe_.end())
                .forward(50)
                .build();
        Trajectory rightat_back = drive.trajectoryBuilder(right_under_truss.end())
                .lineToLinearHeading(new Pose2d(46, -41.2, Math.toRadians(0.00)))
                .build();
        Trajectory rightapproach = drive.trajectoryBuilder(rightat_back.end())
                .forward(4.65)
                .build();
        Trajectory away_right = drive.trajectoryBuilder(rightapproach.end())
                .back(5.8)
                .build();
        Trajectory threeright = drive.trajectoryBuilder(away_right.end())
                .strafeRight(16.5)
                .build();
        Trajectory backstraightright = drive.trajectoryBuilder(threeright.end())
                .back(45)
                .build();
        Trajectory right_park = drive.trajectoryBuilder(backstraightright.end())
                .strafeRight(16)
                .build();
        Trajectory backup_middle = drive.trajectoryBuilder(middle.end())
                .back(2)
                .build();
        Trajectory strafe_middle = drive.trajectoryBuilder(backup_middle.end())
                .strafeLeft(16)
                .build();
        Trajectory stage_middle = drive.trajectoryBuilder(strafe_middle.end())
                .splineTo(new Vector2d(-42.88, -10.68), Math.toRadians(0.00))
                .build();
        Trajectory straight_middle = drive.trajectoryBuilder(stage_middle.end())
                .forward(57)
                .build();
        Trajectory drop_middle = drive.trajectoryBuilder(straight_middle.end())
                .splineTo(new Vector2d(39.17, -34.4), Math.toRadians(0.00))
                .build();
        Trajectory deposit_middle = drive.trajectoryBuilder(drop_middle.end())
                .forward(5.8)
                .build();
        Trajectory away_middle = drive.trajectoryBuilder(deposit_middle.end())
                .back(5.5)
                .build();
        Trajectory middle_after = drive.trajectoryBuilder(away_middle.end())
                .strafeLeft(25)
                .build();
        Trajectory middle_park = drive.trajectoryBuilder(middle_after.end())
                .forward(7.7)
                .build();
        waitForStart();
        if (isStopRequested()) return;
       yellowPixelBoxPipeline.Location location = detector.getLocation();
        switch (location) {
            case LEFT: //left
//


                break;
//
            case RIGHT: //middle
//
                break;

        }




        webcam.stopStreaming();
    }
    public void grab(){

//        intake.intake(2.9);//grab

        intake.outtake(0.5);//grab
        arm.downpixel();//grab
        sleep(1350);//grab
        arm.grab();//grab
        sleep(500);//grab
        arm.drop.setPosition(466);
        arm.aftergrab();//grab

    }
}



