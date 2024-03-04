package CompCode;


//import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Hardware.v2bot_map;


import com.arcrobotics.ftclib.controller.PIDController;

@TeleOp(name="center_tele")
public class coolTele extends LinearOpMode {


    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode

    //TELEmap robot   = new TELEmap();
    v2bot_map robot = new v2bot_map();

    ElapsedTime toggleTimer = new ElapsedTime();
    ElapsedTime toggleTimer2 = new ElapsedTime();
    public PIDController controller;
    public static double p = 0.005, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = 0.0007;
    double toggleTime = .25;
    double toggleTime2 = .25;
    public DcMotorEx larm;
    public DcMotorEx rarm;

    double dropstack = 5;
    double stackpos = 0;

    double FRONTGRAB = .2729;
    double REARGRAB = .413;
    double FRONTRELEASE = .472;
    double REARRELEASE = .235;

    double rotate_pos = 2;

    double drop_pos = 5;

    // used with the dump servo, this will get covered in a bit
//    ElapsedTime toggleTimer = new ElapsedTime();
    //double toggleTime = .25;


    // public static int HIGH = 500; //2900 = HIGH
//        public ElapsedTime runtime = new ElapsedTime();
//    public PIDController controller;
//    public static double p = 0.005, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
//    public static double f = 0.0007;  // prevents arm from falling from gravity
//
//
//    public enum LiftPos {
//        START,
//        LOW,
//        MID,
//        MIDHIGH,
//        LOW_AUTO,
//        HIGH,
//        MANUAL
//    }
//
//    public DcMotorEx larm;
//    public DcMotorEx rarm;
//    public static int START_POS = 0;
//    public static int LOW_POS = 1030;
//    public static int MID_POS = 1460;
//    public static int MID_HIGH_POS = 1750;
//    //max 2320
//    public static int HIGH_POS = 2250;
//    public static int LOW_AUTO = 900;
//    public static int liftTarget = 0; // target position
//
////    int MANUAL = larm.getCurrentPosition() +20;
//
//    private MultipleTelemetry tl;
//
//
////    int MANUAL = larm.getCurrentPosition() +20;
//
//    //private MultipleTelemetry tl;

    double intakepos = 4;
    double waitTime1 = .5;
    double waitTime2 = 0.15;
    double waitTime3 = 1.05;
    double waitTime4 = .65;
    double waitTime6 = .45;
    double waitTime7 = .45;
    private MultipleTelemetry tl;

    ElapsedTime waitTimer1 = new ElapsedTime();
    ElapsedTime waitTimer2 = new ElapsedTime();
    ElapsedTime waitTimer3 = new ElapsedTime();
    ElapsedTime waitTimer4 = new ElapsedTime();

    ElapsedTime waitTimer6 = new ElapsedTime();

    ElapsedTime waitTimer7 = new ElapsedTime();

    ElapsedTime runtime = new ElapsedTime();
//

    double SpeedAdjust = 1;
    public static int liftTarget = 0;
    public static int START_POS = 0;
    public static int LOW_POS = 550;
    public static int MID_POS = 700;
    public static int MID_HIGH_POS = 1000;
    //max 2320
    public static int HIGH_POS = 1750;
    public static int LOW_AUTO = 900;

//    double servospeed = 0.5;
//
//    double lDropPos = 0;
//    double rDropPos = 0;

    enum elbowDownState { //INTAKE
        START,
        MID,
        ALMOST,
        INTAKE,
        WRIST

    }
    enum grab { //INTAKE
        START,
        DOWN,
        PICKPIXELS,
        UP


    }


    enum elbowUpState { //OUTTAKE no lift
        START,
        OUTTAKE,
        WRIST


    }

    enum Outtakelift { //OUTTAKE lift
        START,
        OUTTAKE,
        WRIST


    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        Lift lift = new Lift(hardwareMap, telemetry);
        // Lift lift = new Lift(hardwareMap, telemetry);


        robot.drop.setPosition(.5);
        robot.frontclaw.setPosition(FRONTRELEASE);
        robot.rearclaw.setPosition(REARRELEASE);
        robot.rotwrist.setPosition(.497);
        robot.raxon.setPosition(.665);
        robot.laxon.setPosition(.335);
        robot.bendwrist.setPosition(.1535);

        waitForStart();

        if (isStopRequested()) return;

        elbowUpState outtake = elbowUpState.START;
        elbowDownState intake = elbowDownState.START;
        grab claw = grab.START;
        Outtakelift outlift = Outtakelift.START;
//
//        //LiftPos liftTarget = LiftPos.START;
//        liftTarget = 0;
        while (opModeIsActive() && !isStopRequested()) {

            switch (outlift) { // scoring pos with lift
                case START:
                    if (gamepad2.circle || gamepad2.dpad_down || gamepad2.dpad_up) {
                        robot.drop.setPosition(.465);
                        robot.bendwrist.setPosition(.105);
                        waitTimer1.reset();
                        outtake = elbowUpState.OUTTAKE; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case OUTTAKE:
                    if(waitTimer1.seconds() >= waitTime1) {
                        robot.drop.setPosition(.465);
                        robot.raxon.setPosition(.3);
                        robot.laxon.setPosition(.7);
//                        robot.laxon.setPosition(.92);
                        waitTimer1.reset();
                        outtake = elbowUpState.WRIST;
                    }
                    break;
                case WRIST:
                    if(waitTimer4.seconds() >= waitTime4) {

                        rotate_pos = 2;
                        robot.bendwrist.setPosition(.705);



                        robot.drop.setPosition(.5);

                        waitTimer4.reset();
                        outtake = elbowUpState.START;
                    }
                    break;

            }

            switch (outtake) { // scoring pos no lift
                case START:
                    if (gamepad2.square) {
                        robot.drop.setPosition(.465);
                        robot.bendwrist.setPosition(.145);
                        waitTimer1.reset();
                        outtake = elbowUpState.OUTTAKE; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case OUTTAKE:
                    if(waitTimer1.seconds() >= waitTime1) {
                        robot.drop.setPosition(.465);
                        robot.raxon.setPosition(.3);
                        robot.laxon.setPosition(.7);
//                        robot.raxon.setPosition(.23);
//                        robot.laxon.setPosition(.77);
//
                        waitTimer1.reset();
                        outtake = elbowUpState.WRIST;
                    }
                    break;
                case WRIST:
                    if(waitTimer4.seconds() >= waitTime4) {

                        rotate_pos = 4;
                        robot.bendwrist.setPosition(.682);



                        robot.drop.setPosition(.465);

                        waitTimer4.reset();
                        outtake = elbowUpState.START;
                    }
                    break;

            }

            switch (intake) { // start pos
                case START:
                    if (gamepad2.cross) {
                        rotate_pos = 2;
                        robot.bendwrist.setPosition(.149);
                        robot.frontclaw.setPosition(FRONTRELEASE);
                        robot.rearclaw.setPosition(REARRELEASE);
                        robot.drop.setPosition(.465);
                        waitTimer2.reset();
                        intake = elbowDownState.MID; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case MID:
                    if(waitTimer2.seconds() >= waitTime2) {
                        robot.raxon.setPosition(.55);
                        robot.laxon.setPosition(.45);
//                        robot.raxon.setPosition(.3);
//                        robot.laxon.setPosition(.7);


                        waitTimer2.reset();
                        intake = elbowDownState.INTAKE;
                    }
                    break;
                case ALMOST:
                    if(waitTimer2.seconds() >= waitTime2) {
                        robot.raxon.setPosition(.575);
                        robot.laxon.setPosition(.35);
                        robot.bendwrist.setPosition(.153);
//                        robot.raxon.setPosition(.3);
//                        robot.laxon.setPosition(.7);


                        waitTimer6.reset();
                        intake = elbowDownState.INTAKE;
                    }
                    break;
                    case INTAKE:
                    if(waitTimer6.seconds() >= waitTime6) {
                        robot.raxon.setPosition(.665);
                        robot.laxon.setPosition(.335);
                        robot.drop.setPosition(.5);
                        robot.bendwrist.setPosition(.1525);

                        intake = elbowDownState.START;
                    }
                    break;


            }

            switch (claw) { //grab from transfer
                case START:
                    if (gamepad2.triangle) {
                        robot.frontclaw.setPosition(FRONTRELEASE);
                        robot.rearclaw.setPosition(REARRELEASE);


                        waitTimer6.reset();
                        claw = grab.DOWN; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case DOWN:
                    if(waitTimer6.seconds() >= waitTime6) {
                        robot.raxon.setPosition(.738);
                        robot.laxon.setPosition(.272);
                        robot.bendwrist.setPosition(.15325);
                        robot.drop.setPosition(.465);
                        waitTimer6.reset();
                        claw = grab.PICKPIXELS;
                    }
                    break;
                case PICKPIXELS:
                    if(waitTimer6.seconds() >= waitTime6) {
                        robot.frontclaw.setPosition(FRONTGRAB);
                        robot.rearclaw.setPosition(REARGRAB);

                        waitTimer7.reset();
                        claw = grab.UP;
                    }
                    break;
                case UP:
                    if(waitTimer7.seconds() >= waitTime7) {
                        robot.raxon.setPosition(.64);
                        robot.laxon.setPosition(.36);
                        robot.bendwrist.setPosition(.149);
                        claw = grab.START;
                    }
                    break;

            }




//


            if (gamepad2.circle) { //grab rear
                robot.rearclaw.setPosition(REARGRAB);
            }
            if (gamepad2.square) { //grab front
                robot.frontclaw.setPosition(FRONTGRAB);
            }
            if (gamepad2.left_stick_button) { //release front
                robot.frontclaw.setPosition(FRONTRELEASE);
            }
            if (gamepad2.right_stick_button) { //release rear
                robot.rearclaw.setPosition(REARRELEASE);
            }

            if (gamepad2.right_bumper){
                //toggle
                if (rotate_pos<5) {
                    toggleTimer.reset();
                    rotate_pos++;
                }
                while (opModeIsActive()
                        && (toggleTimer.seconds() < toggleTime)
                ) {

                }

            } else if (gamepad2.left_bumper){
                //toggle L

                if (rotate_pos >1) {
                    toggleTimer.reset();
                    rotate_pos--;
                }
                while (opModeIsActive()
                        && (toggleTimer.seconds() < toggleTime)
                ) {

                }

            }

            if (gamepad1.dpad_down){
                //toggle
                if (intakepos<4) {
                    toggleTimer2.reset();
                    intakepos++;
                }
                while (opModeIsActive()
                        && (toggleTimer2.seconds() < toggleTime2)
                ) {

                }
            }  if (gamepad1.dpad_up){
                //toggle L

                if (intakepos>1) {
                    toggleTimer2.reset();
                    intakepos--;
                }
                while (opModeIsActive()
                        && (toggleTimer2.seconds() < toggleTime2)
                ) {

                }

            }
            if (intakepos == 4 ) {
                drop_pos = .15;
            }
            if (intakepos == 3 ) {
               drop_pos = .4;
            }
            if (intakepos == 2 ) {
               drop_pos = .6;
            }
            if (intakepos == 1) {
                drop_pos = .8;
            }
//            if (intakepos == 4 ) {
//                drop_pos = .5;
//            }
//            if (intakepos == 3 ) {
//                drop_pos = .4;
//            }
//            if (intakepos == 2 ) {
//                drop_pos = .472;
//            }
//            if (intakepos == 1) {
//                drop_pos = .5;
//            }

            if (gamepad1.right_trigger > 0) { //out
                robot.intake.setPower(-3);
                robot.drop.setPosition(.5);


            } else if (gamepad1.left_trigger > 0) { //in
                //servospeed = 0.5;
                robot.drop.setPosition(drop_pos);

                robot.intake.setPower(3);

            }else  {

                robot.intake.setPower(0);

            }





                //start stack
               if (rotate_pos == 1 ) {
                    robot.rotwrist.setPosition(.53);
                }
                 if (rotate_pos == 2) {
                    robot.rotwrist.setPosition(.497); // intake
                }
                if (rotate_pos ==3){
                    robot.rotwrist.setPosition(.4655); // outtake
                }
                 if (rotate_pos == 4){
                    robot.rotwrist.setPosition(.435);
                }
                if (rotate_pos == 5){
                    robot.rotwrist.setPosition(.4);
                }

//            if (rotate_pos == 1 ) {
//                robot.rotwrist.setPosition(.4);
//            }
//            if (rotate_pos == 2) {
//                robot.rotwrist.setPosition(.53);
//            }
//            if (rotate_pos == 3){
//                robot.rotwrist.setPosition(.49);
//            }
//            if (rotate_pos == 4){
//                robot.rotwrist.setPosition(.4655);
//            }
//            if (rotate_pos == 5){
//                robot.rotwrist.setPosition(.435);
//            }




            else if (gamepad2.dpad_down) {
                liftTarget = LOW_POS;

            }
                else if (gamepad2.square) {
                    liftTarget = START_POS;

                }
            else if (gamepad2.dpad_up) {
                liftTarget = HIGH_POS;
            }
            if (gamepad2.cross) {
                liftTarget = START_POS;
            }

            else if (gamepad2.dpad_right) {
                liftTarget = larm.getCurrentPosition() + 40;
            }
            else if (gamepad2.dpad_left) {
                liftTarget = larm.getCurrentPosition() - 40;
            }


            if (gamepad1.left_bumper) {
                SpeedAdjust = 4;
            } else if (gamepad1.right_bumper) {
                SpeedAdjust = 1;
            }

            if (gamepad1.cross) {
               // robot.climb.r(100);

            }
//


            robot.leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
            robot.leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

              if (gamepad2.right_trigger > 0) {
                    robot.climb.setPower(-2);

                }else if (gamepad2.left_trigger > 0) {
                    robot.climb.setPower(2);
                }else {
                    robot.climb.setPower(0);
                }


               telemetry.addData("rotate#:",rotate_pos);
            telemetry.addData("intake#:",intakepos);
              telemetry.update();
              lift.update();
        }
    }
    class Lift {
        public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware

            controller = new PIDController(p, i, d);
            tl = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            larm = hardwareMap.get(DcMotorEx.class, "Llift");
            rarm = hardwareMap.get(DcMotorEx.class, "Rlift");


            larm.setDirection(DcMotorEx.Direction.FORWARD);
            rarm.setDirection(DcMotorEx.Direction.REVERSE);

            larm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            larm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void update() {
//                setTarget(target);
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift


            controller.setPID(p, i, d);

            int larmPos = larm.getCurrentPosition();
            int rarmPos = rarm.getCurrentPosition();

            double Lpid = controller.calculate(larmPos, liftTarget);
            double Rpid = controller.calculate(rarmPos, liftTarget);

            // double Lff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; //* (12/voltageSensor.getVoltage()
            // double Rff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; // * (12/voltageSensor.getVoltage()

            double Lpower = Lpid + f;
            double Rpower = Rpid + f;

            larm.setPower(Lpower);
            rarm.setPower(Rpower);


            tl.update();
        }

        public boolean atTarget() {
            return controller.atSetPoint();
        }

//            public void moveToTarget(LiftPos target) {
//
//                setTarget(target);
//
//                while (!atTarget()) {
//                    update(target);
//
//                }
//            }

//            public void setTarget(LiftPos target) {
//                int encoderTarget = 0;
//                // Beep boop this is the lift update function
//                // Assume this runs some PID controller for the lift
//                switch (target) {
//                    //int MANUAL = larm.getCurrentPosition() +20;
//
//                    case START:
//                        encoderTarget = START_POS;
//                        break;
//                    case LOW:
//                        encoderTarget = LOW_POS;
//                        break;
//
//                    case MID:
//                        encoderTarget = MID_POS;
//                        break;
//                    case MIDHIGH:
//                        encoderTarget = MID_HIGH_POS;
//                        break;
//                    case HIGH:
//                        encoderTarget = HIGH_POS;
//                        break;
//
//                    case LOW_AUTO:
//                        encoderTarget = LOW_AUTO;
//                        break;
//                    case MANUAL:
//                        encoderTarget = larm.getCurrentPosition() + 20;
//                        break;
//                }

//                controller.setSetPoint(encoderTarget);
//            }

    }
}
