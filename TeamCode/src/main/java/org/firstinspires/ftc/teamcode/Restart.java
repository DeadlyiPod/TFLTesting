package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Guard on 1/3/2019.
 */
@Autonomous(name = "Restart",group = "reset")
public class Restart extends LinearOpMode {

    private DcMotor rotateMotor = null;


    @Override
    public void runOpMode() {
         rotateMotor = hardwareMap.get(DcMotor.class, "rotateMotor");

        while (!opModeIsActive()&&!isStopRequested()) {
            telemetry.addData("Status", "Waiting in Init");     telemetry.update(); }

        while (opModeIsActive()){
            rotateMotor.setPower(0.5);
            sleep(800);
            rotateMotor.setPower(0.3);
        }
    }
}
