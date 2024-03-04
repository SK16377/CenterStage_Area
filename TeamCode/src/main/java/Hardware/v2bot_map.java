/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor; //REV

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
@Config
public class v2bot_map {
    /* Public OpMode members. */
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    public DcMotor climb;

    public DcMotorEx intake;
    public Servo drone;
    public Servo wheel;
    public Servo raxon;
    public Servo wrist;
    public Servo laxon;
    public Servo lflap;
    public Servo rflap;


    public Servo rearclaw;
    public Servo frontclaw;
    public Servo drop;

    public Servo rotwrist;
    public Servo bendwrist;
    //public Servo rightaxon;
   // public Servo leftaxon;

    //    public Servo ldrop;
    public WebcamName webcam;
//    public DcMotor rin = null;
//    public DcMotor lin = null;


    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public v2bot_map() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotorEx.class, "parallelEncoder");
        leftBack = hwMap.get(DcMotorEx.class, "perpendicularEncoder");
        rightBack = hwMap.get(DcMotorEx.class, "rb");
        rightFront = hwMap.get(DcMotorEx.class, "rf");
        //drone = hwMap.get(Servo.class, "release");
        climb = hwMap.get(DcMotor.class, "climb");
        intake = hwMap.get(DcMotorEx.class, "intake"); //intake spin
        drop = hwMap.get(Servo.class, "drop"); //intake move up and down
        rearclaw = hwMap.get(Servo.class, "rearclaw"); //tiny claw 1
        frontclaw = hwMap.get(Servo.class, "frontclaw"); //tiny claw 2
        bendwrist = hwMap.get(Servo.class, "bendwrist");
        rotwrist = hwMap.get(Servo.class, "spinwrist");
        laxon = hwMap.get(Servo.class, "laxon");
        raxon = hwMap.get(Servo.class, "raxon");
        raxon = hwMap.get(Servo.class, "raxon");
        lflap = hwMap.get(Servo.class, "lflap");
        rflap = hwMap.get(Servo.class, "rflap");

//        webcam = hwMap.get(WebcamName.class, "Webcam 1");

//        raxon = hwMap.get(Servo.class, "raxon");
//        laxon = hwMap.get(Servo.class, "laxon");
//        wrist = hwMap.get(Servo.class, "wrist");
//        ldrop = hwMap.get(Servo.class, "ldrop");
        //claw1 = hwMap.get(Servo.class, "claw1");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        climb.setDirection(DcMotor.Direction.REVERSE);
//

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        climb.setPower(0);
        intake.setPower(0);

//        lin.setPower(0);
//        rin.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.


    }
}


