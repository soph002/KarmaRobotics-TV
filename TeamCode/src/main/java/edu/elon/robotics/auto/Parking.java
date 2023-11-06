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
        sleep(3000);
        robot.resetDriveEncoders();
        robot.imu.resetYaw();
        double flatWall = robot.distanceSensor.getDistance(DistanceUnit.CM);
        System.out.println("Flat wall"+ flatWall);
        boolean didWePark = false;

        while (robot.colorSensor.alpha()<(robot.maxBrightness-200) && opModeIsActive() && !didWePark) {
            while (robot.distanceSensor.getDistance(DistanceUnit.CM)<(flatWall+5) && opModeIsActive()) {
                robot.motorLeft.setPower(power*.58);
                robot.motorRight.setPower(power*-.58);
                robot.motorAux.setPower(.1*getHeading());
                System.out.println("Distance Traveled"+ robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition()));
                System.out.println("Block Distance"+ robot.distanceSensor.getDistance(DistanceUnit.CM));
            }
            double startDistance = robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition());
            System.out.println("Start distance: "+ startDistance);
            double maxDepth = 0;
            while (robot.distanceSensor.getDistance(DistanceUnit.CM)>=(flatWall+5) && opModeIsActive()) {
                System.out.println("Distance Traveled"+ robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition()));
                System.out.println("Block Distance"+ robot.distanceSensor.getDistance(DistanceUnit.CM));
                if (robot.distanceSensor.getDistance(DistanceUnit.CM) > maxDepth) {
                    maxDepth = robot.distanceSensor.getDistance(DistanceUnit.CM);
                }
            }
            double parkingWidth = robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition()) - startDistance;
            robot.startMove(0,0,0,0);
            if (parkingWidth>=45 && (maxDepth-flatWall>=45)) {
                driveDistance(30,0,-.1);
                while (robot.distanceSensor.getDistance(DistanceUnit.CM)>30 && opModeIsActive()) {
                    System.out.println("Block DistanceParking"+ robot.distanceSensor.getDistance(DistanceUnit.CM));
                    robot.startMove(0,.1,0,0);
                }
                didWePark=true;
            }
            robot.startMove(0,0,0,0);
        }
        robot.startMove(0,0,0,0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        lookingForParking(.2);
    }

}
