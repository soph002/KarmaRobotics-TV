package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import edu.elon.robotics.auto.AutoCommon;

@Autonomous(name="Testing Drive", group="labs")
public class testingAuto extends AutoCommon{

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        // touchMe(.3);
        driveToCalibrateLightSensor(.3);

//        // test turnIMU()
//        turnIMU(90, 0.5);
//        sleep(1000);   // let robot settle before taking a final heading
//        System.out.println("TURNIMU_ " + getHeading());
//
//        turnIMU(-90, 0.5);
//        sleep(1000);
//        System.out.println("TURNIMU_ " + getHeading());
//
//        turnIMU(45, 0.5);
//        sleep(1000);
//        System.out.println("TURNIMU_ " + getHeading());
//
//        turnIMU(-45, 0.5);
//        sleep(1000);
//        System.out.println("TURNIMU_ " + getHeading());
//
//        turnIMU(-180, 0.5);
//        sleep(1000);
//        System.out.println("TURNIMU_ neg" + getHeading());
//
//        turnIMU(180, 0.5);
//        sleep(1000);
//        System.out.println("TURNIMU_ " + getHeading());
    }

}
