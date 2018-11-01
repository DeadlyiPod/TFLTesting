package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private CRServo intake = null;

    // The IMU sensor object
    BNO055IMU imu;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    //Vuforia License
    private static final String VUFORIA_KEY = "AV4rzZr/////AAAAGdd9iX6K6E4vot4iYXx7M+sE9XVwwTL30eOKvSPorcY1yK25A3ZI/ajH4Ktmg+2K1R4sUibLK6BBgw/jKf/juUgjbwB6Wi/magAhEnKorWebeAg8AzjlhbgBE5mhmtkX60bedZF/qX/6/leqVhEd0XZvGn/3xv56Z5NMrOsZzJRMqWNujm4R8Q1fhjBqwIkFuhGzJ2jFzWktAebZcGaImLwgaOjNlYLebS8lxpDuP7bnu/AwsRo/up1zuvUoncDabDS4SFeh/Vjy2fIFApnq7GieBaL2uv4gssG2JUgYvXz3uvQAswf5b5k8v6z0120obXqyH3949gLYeyoY/uZ5g9r93aoyxr2jEwg7+tRezzit";


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

        intake = hardwareMap.get(CRServo.class, "spinCR");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Set Motor Polarities
        leftInnerDrive.setDirection(DcMotor.Direction.REVERSE);
        leftOuterDrive.setDirection(DcMotor.Direction.FORWARD);

        rightInnerDrive.setDirection(DcMotor.Direction.FORWARD);
        rightOuterDrive.setDirection(DcMotor.Direction.REVERSE);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                //Drive power variables
                double leftInnerPower;
                double leftOuterPower;
                double rightInnerPower;
                double rightOuterPower;
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
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
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
            newLeftTarget = leftInnerDrive.getCurrentPosition() + moveCounts;
            newRightTarget = rightInnerDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftInnerDrive.setTargetPosition(newLeftTarget);
            rightInnerDrive.setTargetPosition(newRightTarget);

            leftInnerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightInnerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftInnerDrive.setPower(speed);
            rightInnerDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftInnerDrive.isBusy() && rightInnerDrive.isBusy())) {
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

                leftInnerDrive.setPower(leftSpeed);
                leftOuterDrive.setPower(leftSpeed);
                rightInnerDrive.setPower(rightSpeed);
                rightOuterDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftInnerDrive.getCurrentPosition(),
                        rightInnerDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
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
        leftInnerDrive.setPower(leftSpeed);
        rightInnerDrive.setPower(rightSpeed);
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
