package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by johnb on 9/26/2017.
 */


@TeleOp(name="Main: Tank Drive Train", group="Linear Opmode")

//Comment out @Disabled to add this OpMode to the Driver Station List!
//@Disabled

public class Exp_TankDrive_TeleOp extends LinearOpMode {

    //Declare OpMode members/variables (Motors, servos, etc.)

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor armMotor = null;
    private DcMotor wheelMotor = null;


    private Servo leftGrab = null;
    private Servo rightGrab = null;

    private Servo leftGrab2 = null;
    private Servo rightGrab2 = null;

    private Servo jewelStick = null;




    //Non-static vars

    double leftOpenPos = 0.19;
    double rightOpenPos = 0.93;

    double leftClosePos = 0.75;
    double rightClosePos = 0.33;

    double leftHalfPos = leftClosePos - leftOpenPos;
    double rightHalfPos = rightOpenPos - rightClosePos;

    double leftOpenPos2 = 0.79;
    double rightOpenPos2 = 0.00;

    double leftClosePos2 = 0.30;
    double rightClosePos2 = 0.49;

    double leftHalfPos2 = 0.50;
    double rightHalfPos2 = 0.25;






    boolean grabberClosed;
    boolean grabberClosed2;
    boolean toggleButtonXDown;
    boolean toggleButtonADown;

    boolean toggleYDown;
    boolean wheelUpDown;

    private ElapsedTime runtimeArm = new ElapsedTime();


    static final double     ARM_SPEED             = 0.6;

    //Arm encoder positions
    int armPos1 = 0;
    int armPos2 = 1;
    int armPos3 = 1;

    int minArmPos = -1685;

    //Jewel Stick positions
    double jewelPos1 = 0.85;
    double jewelPos2 = 0.25;






    //private Context context = hardwareMap.appContext;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Initialize the hardware variables. Strings used a parameters to "get"
        // must correspond to the names assigned during robot config on the FTC Robot Controller
        //App
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        leftGrab = hardwareMap.get(Servo.class, "left_grab");
        rightGrab = hardwareMap.get(Servo.class, "right_grab");

        leftGrab2 = hardwareMap.get(Servo.class, "left_grab2");
        rightGrab2 = hardwareMap.get(Servo.class, "right_grab2");

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        jewelStick = hardwareMap.get(Servo.class, "jewel_servo");
        //Rear Wheeled motor
        wheelMotor = hardwareMap.get(DcMotor.class, "rear_push");


        //Reverse the motor that runs backwards when given positive power.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setDirection(DcMotor.Direction.REVERSE);

        leftGrab.setDirection(Servo.Direction.FORWARD);
        rightGrab.setDirection(Servo.Direction.FORWARD);

        leftGrab2.setDirection(Servo.Direction.FORWARD);
        rightGrab2.setDirection(Servo.Direction.FORWARD);

        wheelMotor.setDirection(DcMotor.Direction.FORWARD);




        //Reset Encoders
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Rear Wheel running with Encoders
        wheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d",
                armMotor.getCurrentPosition());



        //Wait for game to start/ Wait for driver to press PLAY
        waitForStart();
        runtime.reset();

        //Runs until the end of the match (Driver presses stop)
        while (opModeIsActive()) {

            //Setup speed variables for each drive Motor/wheel to save power level
            double leftPower;
            double rightPower;

            double armPower;


            int armEncoderPos;




            //Choose to drive using either Tank Mod, or POV Mode.

            //POV Mode uses left stick to go forward, and right stick to turn
            //Uses basic math to combine motions and is easier to drive stright
            /*double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);*/


            //Tank Mode  - Simpler to program, harder to drive straight and slow.
            leftPower = -gamepad1.left_stick_y*0.85;
            rightPower = -gamepad1.right_stick_y*0.85;

            //Tank Mode + Right trigger for optional gas! Fix the issue :P
            leftPower = Range.clip(-gamepad1.left_stick_y + gamepad1.right_trigger + -gamepad1.left_trigger, -1.0, 1.0);
            rightPower = Range.clip(-gamepad1.right_stick_y + gamepad1.right_trigger + -gamepad1.left_trigger, -1.0, 1.0);


            //Optionally scale total speed down to make driving more manageable
            leftPower = Range.scale(leftPower, -1.0, 1.0, -0.5, 0.5);
            rightPower = Range.scale(rightPower, -1.0, 1.0, -0.5, 0.5);

            //Send calculated power (set in Power variables) to the actual motors
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);



            //Jewel Stick controls

            if (gamepad2.dpad_down) {
                jewelStick.setPosition(jewelPos1);

            } else if (gamepad2.dpad_up) {
                jewelStick.setPosition(jewelPos2);
            }



            //Motor Arm controls

            //Encoder controls




            /*if (gamepad2.a) {
                moveArm(ARM_SPEED, armPos1,5);
            } else if(gamepad2.b) {
                moveArm(ARM_SPEED, armPos2,5);
            } else if (gamepad2.y) {
                moveArm(ARM_SPEED, armPos3, 5);
            }*/

            //Analog controls


            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armPower = gamepad2.right_stick_y;
            armPower = Range.scale(armPower, -1.0, 1.0, -0.60, 0.40);



            armEncoderPos = armMotor.getCurrentPosition();

            /*if (armEncoderPos < minArmPos) {
                armPower = 0;
            }*/

            //Send calculated power to arm Motor
            armMotor.setPower(armPower);





            telemetry.addData("ArmMotor", "Current Arm Position = " + armEncoderPos);




            //Servo grabber controls

            if(gamepad2.right_bumper && !grabberClosed) {
                leftGrab.setPosition(leftClosePos);
                rightGrab.setPosition(rightClosePos);

                grabberClosed = true;
            }
            else if (gamepad2.left_bumper && grabberClosed){
                leftGrab.setPosition(leftOpenPos);
                rightGrab.setPosition(rightOpenPos);

                grabberClosed = false;
            }

            // toggle style
            if(gamepad2.x && !toggleButtonXDown && !grabberClosed) {
                leftGrab.setPosition(leftClosePos);
                rightGrab.setPosition(rightClosePos);

                grabberClosed = true;

            }
            else if (gamepad2.x && !toggleButtonXDown && grabberClosed){
                leftGrab.setPosition(leftOpenPos);
                rightGrab.setPosition(rightOpenPos);

                grabberClosed = false;

            }

            toggleButtonXDown = gamepad2.x;


            //Servo grabber controls


            // toggle style
            if(gamepad2.a && !toggleButtonADown && !grabberClosed2) {
                leftGrab2.setPosition(leftClosePos2);
                rightGrab2.setPosition(rightClosePos2);

                grabberClosed2 = true;

            }
            else if (gamepad2.a && !toggleButtonADown && grabberClosed2){
                leftGrab2.setPosition(leftOpenPos2);
                rightGrab2.setPosition(rightOpenPos2);

                grabberClosed2 = false;

            }

            toggleButtonADown = gamepad2.a;





            if(gamepad2.y) {
                leftGrab.setPosition(leftHalfPos);
                rightGrab.setPosition(rightHalfPos);
            }

            if(gamepad2.b) {
                leftGrab2.setPosition(leftHalfPos2);
                rightGrab2.setPosition(rightHalfPos2);
            }


            double wheelMotorPower = gamepad2.right_trigger + -gamepad2.left_trigger;
            wheelMotorPower = Range.scale(wheelMotorPower, -1, 1, -0.25, 0.25);

            wheelMotor.setPower(wheelMotorPower);









            //Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (0.2f), right (0.2f)", leftPower, rightPower);
            telemetry.addData("ArmMotor", "Current Arm Position = " + armEncoderPos);
            telemetry.addData("WheelMotor", "Current Backthingie Ticks = " + wheelMotor.getCurrentPosition());
            telemetry.addData("ServoGrab", "Grabber Closed: " + grabberClosed);
            telemetry.addLine("Robotics is awesome! John was here :P");
            telemetry.addLine("Don't forget your GP ;) ;) #BlameEvan");
            telemetry.update();








        }
    }

    public void moveArm (double speed,
                             int armTarget,
                             double timeoutS) {
        int newArmTarget = armTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            // Determine new target position, and pass to motor controller
            armMotor.setTargetPosition(newArmTarget);

            // Turn On RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtimeArm.reset();
            armMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtimeArm.seconds() < timeoutS) &&
                    (armMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", armTarget);
                telemetry.addData("Path2", "Running at %7d", armMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            armMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
