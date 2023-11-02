package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous(name="Parking", group="labs")
public class Parking extends AutoCommon{

    protected void lookingForParking(double power) {
        driveToCalibrateLightSensor(power);
        while (robot.colorSensor.alpha()<(robot.maxBrightness-200) && opModeIsActive()) {
            robot.startMove(power,0,0,0);
        }
        robot.startMove(0,0,0,0);
        sleep(2000);
        turnIMU(90,power);
        robot.resetDriveEncoders();
        robot.imu.resetYaw();
        while (robot.colorSensor.alpha()<(robot.maxBrightness-200) && opModeIsActive()) {
            robot.motorLeft.setPower(power*.58);
            robot.motorRight.setPower(power*-.58);
            robot.motorAux.setPower(.1*getHeading());
            System.out.println("Distance Traveled"+ robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition()));
            System.out.println("Block Distance"+ robot.distanceSensor.getDistance(DistanceUnit.CM));
        }
        robot.startMove(0,0,0,0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        while (opModeIsActive() && opModeIsActive()) {
            lookingForParking(.2);
        }
    }

}
