package org.firstinspires.ftc.teamcode.cameraTest.HSVTest.Test1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cameraTest.HSVTest.Test1.CVMaster;

@Autonomous(name = "Nicholas Cone Detection Test", group = "concept")

public class ConeDetectorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
//        initialize camera and pipeline
        CVMaster cv = new CVMaster(this);
//      call the function to startStreaming
        cv.observeCone();
        waitForStart();
        while (opModeIsActive()) {
        }
//        stopStreaming
        cv.stopCamera();
    }
}