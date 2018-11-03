package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

/**
 * Created by Guard on 11/1/2018.
 */
@Autonomous(name = "RedDepot", group = "Autonomous")
public class RedDepot extends LinearOpMode {

    Orientation angles;
    Acceleration gravity;

    private double integratedYAxis = 0;
    private double lastRoll = 0;

    //Initialize motors/servos
    private DcMotor leftInnerDrive = null;
    private DcMotor leftOuterDrive = null;

    private DcMotor rightInnerDrive = null;
    private DcMotor rightOuterDrive = null;

    private DcMotor slideMotor = null;
    private DcMotor rotateMotor = null;

    private CRServo intake = null;

    String mineralPlace = "No place detected!";
    // The IMU sensor object
    BNO055IMU imu;

    boolean isDown = false;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.4;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.12;     // Larger is more responsive, but also less stable

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    //Vuforia License
    private static final String VUFORIA_KEY = "AV4rzZr/////AAAAGdd9iX6K6E4vot4iYXx7M+sE9XVwwTL30eOKvSPorcY1yK25A3ZI/ajH4Ktmg+2K1R4sUibLK6BBgw/jKf/juUgjbwB6Wi/magAhEnKorWebeAg8AzjlhbgBE5mhmtkX60bedZF/qX/6/leqVhEd0XZvGn/3xv56Z5NMrOsZzJRMqWNujm4R8Q1fhjBqwIkFuhGzJ2jFzWktAebZcGaImLwgaOjNlYLebS8lxpDuP7bnu/AwsRo/up1zuvUoncDabDS4SFeh/Vjy2fIFApnq7GieBaL2uv4gssG2JUgYvXz3uvQAswf5b5k8v6z0120obXqyH3949gLYeyoY/uZ5g9r93aoyxr2jEwg7+tRezzit";

    private ElapsedTime runtime = new ElapsedTime();

    //Declare Vuforia
    private VuforiaLocalizer vuforia;

    //Declare Tensorflow Lite
    private TFObjectDetector tfod;


    @Override
    public void runOpMode() {
        //Initialize Motors/Servos
        //Initialize hardware variables
        leftInnerDrive = hardwareMap.get(DcMotor.class, "leftInnerDrive");
        leftOuterDrive = hardwareMap.get(DcMotor.class, "leftOuterDrive");

        rightInnerDrive = hardwareMap.get(DcMotor.class, "rightInnerDrive");
        rightOuterDrive = hardwareMap.get(DcMotor.class, "rightOuterDrive");

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        rotateMotor = hardwareMap.get(DcMotor.class, "rotateMotor");

        intake = hardwareMap.get(CRServo.class, "spinCR");

        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftOuterDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOuterDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftOuterDrive.setDirection(DcMotor.Direction.FORWARD);
        rightOuterDrive.setDirection(DcMotor.Direction.REVERSE);

        slideMotor.setDirection(DcMotor.Direction.REVERSE);


        intake.setDirection(CRServo.Direction.REVERSE);

        telemetry.addData("Motors/Servos", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        telemetry.addData("imu", "not initialized");
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("imu", "initialized");
        telemetry.update();
        //Set Motor Polarities


        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        telemetry.addData("Vuforia", "initialized");
        telemetry.update();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addData("tfod", "initialized");
        telemetry.update();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 4) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    mineralPlace = "Left";
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    mineralPlace = "Right";
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    mineralPlace = "Center";
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
            if (mineralPlace.equalsIgnoreCase("Center")){
                telemetry.addData("Hellooo", "hi");
                telemetry.update();
                //Go forward slightly to get out of way of lander and move up to just in front of mineral
                gyroDrive(DRIVE_SPEED,5,0);
                //Set Motor to hover above ground and intake to spin.
                rotateSlide(true,false);
                intake(-1);
                sleep(750);
                //Push gently into the mineral to try and collect it and back up slightly.
                gyroDrive(0.3,30,0);
                intake(0);
                rotateSlide(false,false);
                gyroDrive(0.5,-15,0);
                //Rotate Counter-Clockwise 90 degrees
                gyroTurn(TURN_SPEED,-90);
                /*Drive forward to clear the minerals
                gyroDrive(DRIVE_SPEED,15,-90);
                //Rotate a certain amount of degrees to face wall
                gyroTurn(TURN_SPEED,-45);
                //Drive into wall to align
                gyroDrive(DRIVE_SPEED,15,-45);
                //Back up a lil
                gyroDrive(0.3,-5,-45);
                //rotate clockwise 90 degrees
                gyroTurn(TURN_SPEED,45);
                //drive forward to drop off our team marker
                gyroDrive(DRIVE_SPEED,40,45);
                //drop off team marker

                //back into the crater
                gyroDrive(DRIVE_SPEED,-96,45);
                */
            }else if (mineralPlace.equalsIgnoreCase("Left")){
                //Go forward slightly to get out of way of lander and move up to just in front of mineral
                gyroDrive(DRIVE_SPEED,5,0);
                gyroTurn(TURN_SPEED,-25);
                //Set Motor to hover above ground and intake to spin.
                rotateSlide(true,false);
                intake(-1);
                sleep(750);
                //Push gently into the mineral to try and collect it and back up slightly.
                gyroDrive(0.3,30,0);
                intake(0);
                rotateSlide(false,false);
                gyroDrive(0.5,-15,0);
                //Rotate Counter-Clockwise 90 degrees
                gyroTurn(TURN_SPEED,-90);
                /*Drive forward to clear the minerals
                gyroDrive(DRIVE_SPEED,15,-90);
                //Rotate a certain amount of degrees to face wall
                gyroTurn(TURN_SPEED,-45);
                //Drive into wall to align
                gyroDrive(DRIVE_SPEED,15,-45);
                //Back up a lil
                gyroDrive(0.3,-5,-45);
                //rotate clockwise 90 degrees
                gyroTurn(TURN_SPEED,45);
                //drive forward to drop off our team marker
                gyroDrive(DRIVE_SPEED,40,45);
                //drop off team marker

                //back into the crater
                gyroDrive(DRIVE_SPEED,-96,45);
                */
            }else if (mineralPlace.equalsIgnoreCase("Right")){
                //Go forward slightly to get out of way of lander and move up to just in front of mineral
                gyroDrive(DRIVE_SPEED,5,0);
                gyroTurn(TURN_SPEED,25);
                //Set Motor to hover above ground and intake to spin.
                rotateSlide(true,false);
                intake(-1);
                sleep(750);
                //Push gently into the mineral to try and collect it and back up slightly.
                gyroDrive(0.3,30,0);
                intake(0);
                rotateSlide(false,false);
                gyroDrive(0.5,-15,0);
                //Rotate Counter-Clockwise 90 degrees
                gyroTurn(TURN_SPEED,-90);
                /*Drive forward to clear the minerals
                gyroDrive(DRIVE_SPEED,15,-90);
                //Rotate a certain amount of degrees to face wall
                gyroTurn(TURN_SPEED,-45);
                //Drive into wall to align
                gyroDrive(DRIVE_SPEED,15,-45);
                //Back up a lil
                gyroDrive(0.3,-5,-45);
                //rotate clockwise 90 degrees
                gyroTurn(TURN_SPEED,45);
                //drive forward to drop off our team marker
                gyroDrive(DRIVE_SPEED,40,45);
                //drop off team marker

                //back into the crater
                gyroDrive(DRIVE_SPEED,-96,45);
                */
            }

        }
        if (tfod != null) {
            tfod.shutdown();
        }

    }

    //Rotate Slider Method
    private void rotateSlide(boolean down,
                             boolean hover) {



        if(opModeIsActive()){
            if(down) {
                //raise up
                rotateMotor.setPower(-0.5);
                //let it go down
                sleep(550);
                rotateMotor.setPower(0);
            }else if (!down){
                rotateMotor.setPower(0.5);
                sleep(500);
                rotateMotor.setPower(0);
            }
            while (opModeIsActive() && rotateMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to: idk");
                telemetry.addData("Path2",  "Running at: " + rotateMotor.getCurrentPosition());
                telemetry.update();
            }

            //if hover is true then try and hold intake up a bit. Otherwise, set to 0 power.
            if(hover){
                rotateMotor.setPower(0.4);
                sleep(200);
                rotateMotor.setPower(0.2);
            }else {
                rotateMotor.setPower(0);
            }


            //Go back to run using an encoder.
            rotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    //Rotate Intake Method
    private void intake (double speed){
        if(opModeIsActive()){
            intake.setPower(speed);
        }
    }
    //Init Vuforia method
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }


    //Init Tensorflow method
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = (leftInnerDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
            newRightTarget = rightInnerDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftInnerDrive.setTargetPosition(newLeftTarget);
            rightInnerDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftInnerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightInnerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftOuterDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightOuterDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftInnerDrive.setPower(Math.abs(speed));
            rightInnerDrive.setPower(Math.abs(speed));
            leftOuterDrive.setPower(Math.abs(speed));
            rightOuterDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftInnerDrive.isBusy() && rightInnerDrive.isBusy() && rightOuterDrive.isBusy() && leftOuterDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftInnerDrive.getCurrentPosition(),
                        rightInnerDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftInnerDrive.setPower(0);
            rightInnerDrive.setPower(0);
            leftOuterDrive.setPower(0);
            rightOuterDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftInnerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightInnerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftOuterDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightOuterDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = leftOuterDrive.getCurrentPosition() + moveCounts;
            newRightTarget = rightOuterDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftOuterDrive.setTargetPosition(newLeftTarget);
            rightOuterDrive.setTargetPosition(newRightTarget);

            leftOuterDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightOuterDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftOuterDrive.setPower(speed);
            rightOuterDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftOuterDrive.isBusy() && rightOuterDrive.isBusy())) {
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftOuterDrive.setPower(leftSpeed);
                rightOuterDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftOuterDrive.getCurrentPosition(),
                        rightOuterDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftOuterDrive.setPower(0);
            rightOuterDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftOuterDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightOuterDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("Yaw", angles.firstAngle);
            telemetry.update();
        }
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -(rightSpeed);
        }

        // Send desired speeds to motors.
        leftOuterDrive.setPower(leftSpeed);
        rightOuterDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    double getIntegratedYAxis() {

        double newRoll = angles.firstAngle;

        double deltaRoll = newRoll - lastRoll;

        if (deltaRoll < -180) {
            deltaRoll += 360;
        } else if (deltaRoll >= 180) {
            deltaRoll -= 360;
        }
        integratedYAxis = integratedYAxis + deltaRoll;

        lastRoll = newRoll;

        return integratedYAxis;
    }
    void resetYAxis() {

        lastRoll = angles.firstAngle;
        integratedYAxis = 0;
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
