package Autos;

import android.graphics.Canvas;
import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import Hardware.v2bot_map;

import java.util.List;
import java.util.concurrent.TimeUnit;

import Hardware.CenterstageDetector;
import Hardware.CenterstageProcessor;

//import com.acmerobotics.roadrunner.geometry.Pose2d;


//import com.acmerobotics.roadrunner.trajectoryBuilder;

//@Disabled
@TeleOp(name="APRIL_BLUE_BACK", group="Auto")
public class TestAprilTagDrop_BLUE extends LinearOpMode {
    SampleMecanumDrive drive;
    v2bot_map robot = new v2bot_map();
    OpenCvCamera webcam;
   // Lift lift;
   // Arm arm;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    //private CenterstageProcessor detector;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
//        lift = new Lift(hardwareMap, telemetry);
//        arm = new Arm(hardwareMap, telemetry);


        // visionPortal.resumeStreaming();
         Pose2d startPose = new Pose2d(16.79, -64.32, Math.toRadians(0.00));
         drive.setPoseEstimate(startPose);
        initAprilTag();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                // CenterstageProcessor.Location location = detector.getLocation();
                telemetryAprilTag();
                telemetry.update();
//        visionPortal.setProcessorEnabled(detector, false);

                setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
                //sleep(10000);
//               robot.leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) );
//                robot.rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) );
//                robot.leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) );
//                robot.rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) );
////
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
                drive.update();
            }

        }
    }
//    p

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(622.001, 622.001, 319.803, 241.251)
                // ... these parameters are fx, fy, cx, cy.
                .build();
        //detector = new CenterstageProcessor(telemetry);
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
       // builder.setCameraResolution(new Size(320, 240));


        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessors(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        //visionPortal.setProcessorEnabled(aprilTag, true);
       // visionPortal.setProcessorEnabled(detector, false);

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    /*
        Manually set the camera gain and exposure.
        This can only be called AFTER calling initAprilTag(), and only works for Webcams;
       */

       public double[] transformCoordinates(double tagX, double tagY) {
            double robotX = drive.getPoseEstimate().getX();
            double robotY = drive.getPoseEstimate().getY();
            double theta = drive.getPoseEstimate().getHeading();
            double cosTheta = Math.cos(theta);
            double sinTheta = Math.sin(theta);

            double tagGlobalX = robotX + tagX * cosTheta - tagY * sinTheta;
            double tagGlobalY = robotY + tagX * sinTheta + tagY * cosTheta;

            return new double[]{tagGlobalX, tagGlobalY};
       }

        // public double[] transformCoordinates(double aprilPoseX, double aprilPoseY) {
        //     double theta = drive.getPoseEstimate().getHeading();
        //     double cosTheta = Math.cos(theta);
        //     double sinTheta = Math.sin(theta);
        //     double xWorld = drive.getPoseEstimate().getX();
        //     double yWorld = drive.getPoseEstimate().getY();

        //     double[][] tWorldToRobotArray = {
        //             { cosTheta, sinTheta, -(cosTheta * xWorld + sinTheta * yWorld) },
        //             { -sinTheta, cosTheta, -(-sinTheta * xWorld + cosTheta * yWorld) },
        //             { 0, 0, 1 }
        //     };

        //     double[][] aprilMatrixArray = {
        //             { aprilPoseX },
        //             { aprilPoseY },
        //             { 1 }
        //     };

        //     double[][] resultArray = matrixMultiply(tWorldToRobotArray, aprilMatrixArray);

        //     double xRobot = resultArray[0][0];
        //     double yRobot = resultArray[1][0];

        //     return new double[] { xRobot, yRobot };
        // }

        // // Helper method to multiply matrices
        // private static double[][] matrixMultiply(double[][] a, double[][] b) {
        //     int aRows = a.length;
        //     int aCols = a[0].length;
        //     int bCols = b[0].length;
        //     double[][] result = new double[aRows][bCols];

        //     for (int i = 0; i < aRows; i++) {
        //         for (int j = 0; j < bCols; j++) {
        //             for (int k = 0; k < aCols; k++) {
        //                 result[i][j] += a[i][k] * b[k][j];
        //             }
        //         }
        //     }

        //     return result;
        // }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void telemetryAprilTag() {
        double aprilTagGlobalX, aprilTagGlobalY;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.addLine(String.format("ROBOT XYH %6.1f %6.1f %6.1f  (inch, inch, deg)", drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));
                // double[] tagGlobalCoord = transformCoordinates(detection.ftcPose.x, detection.ftcPose.y);
                // telemetry.addLine(String.format("APRIL TAG LOCATION X: %6.1f Y: %6.1f (inch, inch)", tagGlobalCoord[0], tagGlobalCoord[1]));
                // lookup simple implementation of proportinal control loop
                // if the bearing < 0
                //  turn towards it (probably to the right?)
                // else
                //  turn the other way
                // if range > some threshold value
                //  go max speed
                // else
                //  go max speed * some multiplier constant * range
                // if yaw > 0
                //  rotate in place one way
                // else
                //  rotate the other way
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}



