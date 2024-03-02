package sensorTesting;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;

//TODO:
// REV sensors can detect color and distance.
// Will the pixels be directly on top of the sensor?
// Can we use that to detect the pixels instead of specific colors?
// Do we want to use this sensor data to trigger other parts of the robot?

@TeleOp(name = "colorDetect", group = "Sensor")
//@Disabled
public class SensowrColor extends LinearOpMode {

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    double factor = 255;

    @Override
    public void runOpMode() {

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color");

        // array for hue, saturation, and value info
        float hsvValues[] = {0F, 0F, 0F};

        // multiplies raw RGB values with a scale factor to amplify measured values
        final double SCALE_FACTOR = factor;

        waitForStart();

        // loop and read the RGB and distance data.
        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            //affects scale factor and may adjust output numbers/ sensor sensitivity?
            if (gamepad1.dpad_up) {
                factor = factor + 7;
            }

            if (gamepad1.dpad_down) {
                factor = factor - 7;
            }


                //returns distance and color values?
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            //is there a pixel?? this is based on the distance reading
            if (sensorDistance.getDistance(DistanceUnit.CM) < 1) {

                telemetry.addData("Pixel Status", "IN");
            } else {

                telemetry.addData("Pixel Status", "NOT FOUND");
            }
            // a similar argument can be used to find them using colors
            // once we define the actual color values of the pixels

            telemetry.update();
        }

//
    }
}