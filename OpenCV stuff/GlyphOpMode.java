
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.*;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.IOException;

import static android.os.SystemClock.sleep;


@TeleOp(name="DogeCV Glyph Detector", group="DogeCV")
@Disabled
public class GlyphOpMode extends OpMode {

    // Declare OpMode members.
    /* Declare OpMode members. */
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware

    private ElapsedTime runtimeArm = new ElapsedTime();

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private DcMotor armMotor = null;


    private Servo leftGrab = null;
    private Servo rightGrab = null;

    //Jewels stuff
    private Servo jewelStick = null;

    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;

    //Non-static vars

    double leftOpenPos = 0.19;
    double rightOpenPos = 0.93;

    double leftClosePos = 0.75;
    double rightClosePos = 0.33;

    //Jewel Stick positions
    double jewelPos1 = 0.85;
    double jewelPos2 = 0.25;

    boolean grabberClosed;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Neverest Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.15;


    //VUFORIA VARIABLES
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;
    private ElapsedTime runtime = new ElapsedTime();


    private GlyphDetector glyphDetector = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        glyphDetector = new GlyphDetector();
        glyphDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        glyphDetector.minScore = 1;
        glyphDetector.downScaleFactor = 0.3;
        glyphDetector.speed = GlyphDetector.GlyphDetectionSpeed.SLOW;
        glyphDetector.enable();


    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized. Gyro Calibration");
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        leftGrab = hardwareMap.get(Servo.class, "left_grab");
        rightGrab = hardwareMap.get(Servo.class, "right_grab");

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");

        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);


        jewelStick = hardwareMap.get(Servo.class, "jewel_servo");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        //array that *will* hold the hue saturation, and value (HSV)
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;


        boolean colorIsRed = false;


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
        telemetry.update();


        //SET UP VUFORIA
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AV4rzZr/////AAAAGdd9iX6K6E4vot4iYXx7M+sE9XVwwTL30eOKvSPorcY1yK25A3ZI/ajH4Ktmg+2K1R4sUibLK6BBgw/jKf/juUgjbwB6Wi/magAhEnKorWebeAg8AzjlhbgBE5mhmtkX60bedZF/qX/6/leqVhEd0XZvGn/3xv56Z5NMrOsZzJRMqWNujm4R8Q1fhjBqwIkFuhGzJ2jFzWktAebZcGaImLwgaOjNlYLebS8lxpDuP7bnu/AwsRo/up1zuvUoncDabDS4SFeh/Vjy2fIFApnq7GieBaL2uv4gssG2JUgYvXz3uvQAswf5b5k8v6z0120obXqyH3949gLYeyoY/uZ5g9r93aoyxr2jEwg7+tRezzit";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        ClosableVuforiaLocalizer vuforia = new ClosableVuforiaLocalizer(parameters);


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        //MORE VUFORIA INITS
        relicTrackables.activate();

        char caseVumark = 'U';


        //START ACTIONS
        leftGrab.setPosition(leftClosePos);
        rightGrab.setPosition(rightClosePos);

        moveArm(DRIVE_SPEED, -500, 10.0);


        //Knocking off the Jewel (BLUE ALLIANCE)!!!


        //NOTE:SWIVEL ARM DOWN
        jewelStick.setPosition(jewelPos1);
        sleep(1000);

        //COLOR SENSOR FINDS COLOR

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        if (hsvValues[0] < 50 || hsvValues[0] > 330) {                           //If RED
            colorIsRed = true;

        } else if (hsvValues[0] > 150 && hsvValues[0] < 285) {                     //If Blue
            colorIsRed = false;
        } else {
            colorIsRed = false;
        }


        //JEWEL SENSOR PATHS
        if (colorIsRed) {
            //NOTE: SENSOR FACES BACKWARDS
            encoderDrive(0.13, 3, 3, 4.0);
            sleep(500);
            //SWIVEL ARM UP
            jewelStick.setPosition(jewelPos2);

        } else if (!colorIsRed) {
            encoderDrive(DRIVE_SPEED, -4, -4, 4.0);
            //SWIVEL ARM UP
            jewelStick.setPosition(jewelPos2);
            encoderDrive(DRIVE_SPEED, 4, 4, 4.0);
        }

        //Non-Vumark Path
        //Pick up front arm.
        moveArm(DRIVE_SPEED * 1.5, 2500, 5);
        //Get off balance stone
        encoderDrive(DRIVE_SPEED, 29, 29, 10.0);
        //Align with balance stone
        encoderDrive(DRIVE_SPEED, -13, -13, 5.0);
        sleep(250);

        //BEGIN OPENCV STUFF
        vuforia.close();
        //Sleep to allow it to close
        sleep(1000);
        telemetry.addData("Status", "Initialized");


        glyphDetector = new GlyphDetector();
        glyphDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        glyphDetector.minScore = 1;
        glyphDetector.downScaleFactor = 0.3;
        glyphDetector.speed = GlyphDetector.GlyphDetectionSpeed.SLOW;
        glyphDetector.enable();

        double ChosenGlyph = glyphDetector.getChosenGlyphOffset();

        if (ChosenGlyph >= 201){
            encoderDrive(DRIVE_SPEED,-5,-5,10);
        } else if (ChosenGlyph <= 200){
            encoderDrive(DRIVE_SPEED,10,10,10);
        }




        //RELEASE AND BACK UP
        sleep(500);
        leftGrab.setPosition(leftOpenPos);
        rightGrab.setPosition(rightOpenPos);
        sleep(500);     // pause for servos to move

        encoderDrive(DRIVE_SPEED, -2, -2, 3.0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(999999999);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


    @Override
    public void start() {
        runtime.reset();


    }

    @Override
    public void loop() {


        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Glyph Pos X", glyphDetector.getChosenGlyphOffset());
        telemetry.addData("Glyph Pos Offest", glyphDetector.getChosenGlyphPosition().toString());


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        glyphDetector.disable();
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;


        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftTarget = (leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
        newRightTarget = rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.


        // Display it for the driver.
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
        telemetry.addData("Path2", "Running at %7d :%7d",
                leftDrive.getCurrentPosition(),
                rightDrive.getCurrentPosition());
        telemetry.update();


        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move
    }



    public void moveArm (double speed,
                         int armTarget,
                         double timeoutS) {
        int newArmTarget = armTarget;

        // Ensure that the opmode is still active


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


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", armTarget);
                telemetry.addData("Path2", "Running at %7d", armMotor.getCurrentPosition());
                telemetry.update();


            // Stop all motion;
            armMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}

