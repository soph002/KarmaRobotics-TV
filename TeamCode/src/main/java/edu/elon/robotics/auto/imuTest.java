package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import edu.elon.robotics.auto.AutoCommon;

@Autonomous(name="imu Test", group = "labs")

public class imuTest extends AutoCommon {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        // Create an object to receive the IMU angles
        YawPitchRollAngles robotOrientation;

        // Some variables to hold the three separate rotation directions
        double yaw;
        double pitch;
        double roll;

        waitForStart();

        while (opModeIsActive()) {

            // You need to call this to fill the robotOrientation object
            // This needs to be done each time you want a new reading
            // which is why it is called inside the loop.
            // It will not update by itself
            robotOrientation = robot.imu.getRobotYawPitchRollAngles();

            // Then use these methods to extract each angle that you want
            // from the robotOrientation object.
            // This needs to be done each time you want a new reading and after you have done
            //           robotOrientation = robot.imu.getRobotYawPitchRollAngles();
            // They will not update by themselves
            yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            roll  = robotOrientation.getRoll(AngleUnit.DEGREES);

            telemetry.addData("roll", roll);
            telemetry.addData("pitch", pitch);
            telemetry.addData("yaw", yaw);
            telemetry.update();
            sleep(1000);  // Limit the updates to the phone to every second
        }
    }
}