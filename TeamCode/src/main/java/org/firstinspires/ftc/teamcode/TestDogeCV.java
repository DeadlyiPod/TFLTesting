package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by Bobby on 2/5/2019.
 */
@TeleOp(name="TestDogeCV")
public class TestDogeCV extends LinearOpMode {

    //Vuforia License
    private static final String VUFORIA_KEY = "Ae1Vij//////AAABmfqdtDgNtUaAitOrWDSvx255sec+JeO+Vv6LYA2lzXY71IcC9TTEMaGIluQoa3dE/EVeew84ds52ax+4Z1VPtIkLtAmeJLMP3jrGozxT/CwJ0mSZIYawEIbMzCs+0uggrQMdqrTzsThWWSoFpT22scuqHwsDe+hwmii9ARU4KJzeriU3kAqTT+ezKr26CiJ2RQpqc/oif3VwzOughRA06cT9XhuL3NGdt2t3ra6csD4cldy06+5Sujl1G9/XowVSU7kqXmUuqZdGTXQhkmVv1kCSw+OpuyxfQJKoZJ8OMTr/9uXQAVOrFzKuvRTaHpaSgeYWfn/AC3oeYl79YYpafRDI35j2Uq1c4ezJ4GVXdHzk";

    Dogeforia vuforia2;
    WebcamName webcamName;

    GoldDetector detector;


    public void runOpMode(){
        //All DogeCV Stuff until runtime
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();
        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParameters.fillCameraMonitorViewParent = true;
        vuforiaParameters.cameraName = webcamName;
        vuforia2 = new Dogeforia(vuforiaParameters);
        vuforia2.enableConvertFrameToBitmap();

        //Set up Gold detector
        detector = new GoldDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.downscale = 1;
        detector.setSpeed(DogeCV.DetectionSpeed.VERY_SLOW);

        detector.maxAreaScorer.weight = 0.01;
        detector.ratioScorer.weight = 15;

        vuforia2.setDogeCVDetector(detector);
        vuforia2.enableDogeCV();
        vuforia2.showDebug();
        vuforia2.start();

        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");     telemetry.update(); }

        if(opModeIsActive()){
            int goldMineralX = -1;
            int goldMineralY = -1;
            boolean isFound = false;

            while(!isFound && opModeIsActive()){
                telemetry.addData("X Value: ", goldMineralX);
                telemetry.addData("Y Value: ", goldMineralY);
                telemetry.addData("Direct X Value: ", detector.getxValue());
                telemetry.addData("Direct Y Value: ", detector.getyValue());
                telemetry.update();
                goldMineralX = detector.getxValue();
                goldMineralY = detector.getyValue();
            }



        }



    }



}
