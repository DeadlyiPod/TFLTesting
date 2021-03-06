package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
@Autonomous(name = "HalfCrater", group = "Autonomous")
public class HalfCrater extends LinearOpMode {

    Orientation angles;
    Acceleration gravity;

    private double integratedYAxis = 0;
    private double lastRoll = 0;
    double currentAngle = 0;

    //Initialize motors/servos
    private DcMotor leftInnerDrive = null;
    private DcMotor leftOuterDrive = null;

    private DcMotor rightInnerDrive = null;
    private DcMotor rightOuterDrive = null;

    private DcMotor slideMotor = null;
    private DcMotor rotateMotor = null;

    private DcMotor hangMotor = null;

    private CRServo intake = null;
    private Servo outtake = null;
    private Servo depotDrop = null;

    double outTakePos0 = 0.95;
    double outTakePos1 = 0.65;
    double outTakePos2 = 0.37;

    String mineralPlace = "No place detected";

    // The IMU sensor object
    BNO055IMU imu;

    boolean isDown = false;

    //Encoder Drive Settings
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.

    //Gyroscopic Drive Settings
    static final double     HEADING_THRESHOLD       = 2 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.03;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable

    //Vuforia files?
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    //Vuforia License
    private static final String VUFORIA_KEY = "Ae1Vij//////AAABmfqdtDgNtUaAitOrWDSvx255sec+JeO+Vv6LYA2lzXY71IcC9TTEMaGIluQoa3dE/EVeew84ds52ax+4Z1VPtIkLtAmeJLMP3jrGozxT/CwJ0mSZIYawEIbMzCs+0uggrQMdqrTzsThWWSoFpT22scuqHwsDe+hwmii9ARU4KJzeriU3kAqTT+ezKr26CiJ2RQpqc/oif3VwzOughRA06cT9XhuL3NGdt2t3ra6csD4cldy06+5Sujl1G9/XowVSU7kqXmUuqZdGTXQhkmVv1kCSw+OpuyxfQJKoZJ8OMTr/9uXQAVOrFzKuvRTaHpaSgeYWfn/AC3oeYl79YYpafRDI35j2Uq1c4ezJ4GVXdHzk";

    //Declare Vuforia
    private VuforiaLocalizer vuforia;

    //Declare Tensorflow Lite
    private TFObjectDetector tfod;



    //LEAD SCREW VARS
    //Lead Screw Lift Targets
    int liftTargetUp = 18000;
    int liftTargetDown = 0;

    //Servo Position Vars
    double upPos = 1;
    double downPos = 0;

    /* Declare OpMode members. */
    private DcMotor liftMotor = null;

    static final double     LIFT_COUNTS_PER_MOTOR_REV    = 560;    // eg: TETRIX Motor Encoder
    static final double     GEAR_REDUCTION          = 1.25 ;     // This is < 1.0 if geared UP
    static final double     LEAD_ROD_THREAD_MM      = 8.0 ;     // For figuring circumference
    static final double     COUNTS_PER_MILLIMETER   = ((LIFT_COUNTS_PER_MOTOR_REV * GEAR_REDUCTION)/LEAD_ROD_THREAD_MM);
    static final double     LIFT_SPEED              = 0.6;
    static final double     TURN_SPEED              = 0.5;


    private ElapsedTime runtime;



    @Override
    public void runOpMode() {
        runtime = new ElapsedTime();
        //Initialize Motors/Servos
        //Initialize hardware variables
        leftInnerDrive = hardwareMap.get(DcMotor.class, "leftInnerDrive");
        leftOuterDrive = hardwareMap.get(DcMotor.class, "leftOuterDrive");

        rightInnerDrive = hardwareMap.get(DcMotor.class, "rightInnerDrive");
        rightOuterDrive = hardwareMap.get(DcMotor.class, "rightOuterDrive");

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        rotateMotor = hardwareMap.get(DcMotor.class, "rotateMotor");

        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "hangMotor");

        intake = hardwareMap.get(CRServo.class, "Intake");
        outtake = hardwareMap.get(Servo.class, "outTake");
        depotDrop = hardwareMap.get(Servo.class, "depotDrop");

        rotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftOuterDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOuterDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftInnerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightInnerDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftOuterDrive.setDirection(DcMotor.Direction.REVERSE);
        rightOuterDrive.setDirection(DcMotor.Direction.FORWARD);
        leftInnerDrive.setDirection(DcMotor.Direction.REVERSE);
        rightInnerDrive.setDirection(DcMotor.Direction.FORWARD);

        slideMotor.setDirection(DcMotor.Direction.REVERSE);


        intake.setDirection(CRServo.Direction.REVERSE);

        liftMotor.setDirection((DcMotor.Direction.FORWARD));

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d",
                liftMotor.getCurrentPosition());
        telemetry.update();

        runtime.reset();
        /** Wait for the game to begin */
        /** Activate Tensor Flow Object Detection. */

        if (tfod != null) {
            tfod.activate();
        }
        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");     telemetry.update(); }

        if (opModeIsActive()) {
            telemetry.addData("Change: ", "Rotate Motor set power 0.3");
            telemetry.update();
            rotateMotor.setPower(0.3);
            depotDrop.setPosition(upPos);

            //Land first
            encoderLift(true,1,10);





            telemetry.addData("Change: ", "10 Inches forward");
            telemetry.update();
            gyroDrive(0.5,10,0);

            telemetry.addData("Change: ", "Turn to -45 degrees");
            telemetry.update();
            gyroTurn(0.3,-45);


            telemetry.addData("Change: ", "Starting loop");
            telemetry.update();
            int goldMineralX = -1;
            int goldMineralY = -1;
            boolean isFound = false;
            leftOuterDrive.setPower(-0.15);
            rightOuterDrive.setPower(0.15);


            while((goldMineralX > 450 || goldMineralX < 270) && opModeIsActive()){
                // Rotate a lil to the left each time (Moved to top because it will
                // check the position of the gold mineral before moving.)
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();

                telemetry.addData("Current angle: ", angles.firstAngle);
                telemetry.addData("Current GoldX: ", goldMineralX);
                telemetry.addData("Current GoldY: ", goldMineralY);
                telemetry.update();

                //If they're not detected after a certain angle, go back and re-scan until you can find one.
                if (angles.firstAngle > 45){
                    leftOuterDrive.setPower(0.1);
                    rightOuterDrive.setPower(-0.1);
                }else if(angles.firstAngle < -45){
                    leftOuterDrive.setPower(-0.1);
                    rightOuterDrive.setPower(0.1);
                }


                if(tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if(updatedRecognitions != null){
                        for(Recognition recognition : updatedRecognitions){
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)){
                                telemetry.addData("Gold Mineral Found: ", isFound);
                            }else{
                                telemetry.addData("Gold Mineral Found: ", isFound);
                            }
                            telemetry.update();
                            if(recognition.getTop() > 120 ) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                    goldMineralY = (int) recognition.getTop();
                                    isFound = true;

                                }
                            }
                        }
                        //gyroTurn(0.75,currentAngle);
                    }
                }
            }
            //Turn off all motors
            leftInnerDrive.setPower(0);
            leftOuterDrive.setPower(0);
            rightInnerDrive.setPower(0);
            rightOuterDrive.setPower(0);
            sleep(150);
            currentAngle = angles.firstAngle;

            //Drive past minerals based on where the robot is located
            if(currentAngle >= 20){
                telemetry.addData("Guess: ", "Left Side");
                telemetry.update();
                //left side
                //Drive into the mineral
                encoderDrive(1,22,22,10);

            }else if(currentAngle <= -20){
                telemetry.addData("Guess: ", "Right Side");
                telemetry.update();
                //right side
                //Drive into the mineral
                encoderDrive(1,21,20,10);

            }else{
                telemetry.addData("Guess: ", "Center");
                telemetry.update();
                //Assume center
                //Drive into the mineral
                encoderDrive(1,19,18,10);
            }

            rotateSlide(true);
            depotDrop.setPosition(downPos);



        }
        if (tfod != null) {
            tfod.shutdown();
        }

    }

    //Rotate Slider Method
    private void rotateSlide(boolean down) {



        if(opModeIsActive()){
            if(down) {
                //raise up
                rotateMotor.setPower(-0.3);
                //let it go down
                sleep(500);
                rotateMotor.setPower(0);
            }else if (!down){
                rotateMotor.setPower(0.6);
                sleep(800);
                rotateMotor.setPower(0.3);
            }
            while (opModeIsActive() && rotateMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to: idk");
                telemetry.addData("Path2",  "Running at: " + rotateMotor.getCurrentPosition());
                telemetry.update();
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
    private void timeDrive(double drivePower, long seconds, boolean forward){
        if(forward){
            leftOuterDrive.setPower(drivePower);
            leftInnerDrive.setPower(drivePower);
            rightOuterDrive.setPower(drivePower);
            rightInnerDrive.setPower(drivePower);
            sleep(seconds * 1000);
        }else if(!forward){
            leftOuterDrive.setPower(-drivePower);
            leftInnerDrive.setPower(-drivePower);
            rightOuterDrive.setPower(-drivePower);
            rightInnerDrive.setPower(-drivePower);
            sleep(seconds * 1000);
        }

        leftOuterDrive.setPower(0);
        leftInnerDrive.setPower(0);
        rightOuterDrive.setPower(0);
        rightInnerDrive.setPower(0);
    }
    //Init Vuforia method
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }


    //Init Tensorflow method
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.5;
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
            newLeftTarget = (leftOuterDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
            newRightTarget = rightOuterDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftOuterDrive.setTargetPosition(newLeftTarget);
            rightOuterDrive.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            leftOuterDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightOuterDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
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
                    (leftOuterDrive.isBusy() && rightOuterDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftOuterDrive.getCurrentPosition(),
                        rightOuterDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftOuterDrive.setPower(0);
            rightOuterDrive.setPower(0);

            // Turn off RUN_TO_POSITION
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



    public void encoderLift(boolean raiseUp, double speed,
                            double timeoutS) {
        int newLiftTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            if (raiseUp == true) {
                newLiftTarget = liftTargetUp;
            } else if (raiseUp == false) {
                newLiftTarget = liftTargetDown;
            } else {
                newLiftTarget = liftTargetDown;
            }

            liftMotor.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            liftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (liftMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", newLiftTarget);
                telemetry.addData("Path2", "Running at %7d",
                        liftMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            liftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
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
