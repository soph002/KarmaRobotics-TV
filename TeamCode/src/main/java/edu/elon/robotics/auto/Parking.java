package edu.elon.robotics.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous(name="Parking", group="labs")
public class Parking extends AutoCommon{

    protected void lookingForParking(double power) {
        driveToCalibrateLightSensor(power);
        while (robot.colorSensor.alpha()<(robot.maxBrightness-200) && opModeIsActive()) {
            robot.imu.resetYaw();
            robot.motorLeft.setPower(power*.58);
            robot.motorRight.setPower(power*-.58);
            robot.motorAux.setPower(.1*getHeading());
        }
        robot.startMove(0,0,0,0);
        sleep(2000);
        turnIMU(90,power);
        sleep(2000);
        robot.resetDriveEncoders();
        double flatWall = robot.distanceSensor.getDistance(DistanceUnit.CM);
        System.out.println("Flat wall"+ flatWall);
        boolean didWePark = false;
        System.out.println("max brightness "+ robot.maxBrightness);
        double minimumSpotLength = 10000000;
        double startDistance=0;
        double maxDepth=0;
        double distanceToGoBack=0;
        while (robot.colorSensor.alpha()<(robot.maxBrightness-200) && opModeIsActive()) {
            System.out.println("current brightness "+ robot.colorSensor.alpha());
            while (robot.distanceSensor.getDistance(DistanceUnit.CM)<=(flatWall+10) && opModeIsActive() && robot.colorSensor.alpha()<(robot.maxBrightness-200)) {
                robot.imu.resetYaw();
                robot.motorLeft.setPower(power*.58);
                robot.motorRight.setPower(power*-.58);
                robot.motorAux.setPower(.1*getHeading());
                System.out.println("Distance Traveled"+ robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition()));
                System.out.println("Block Distance"+ robot.distanceSensor.getDistance(DistanceUnit.CM));
            }
            startDistance = robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition());
            System.out.println("Start distance: "+ startDistance);

            double currentSpotLength = 0;
            while (robot.distanceSensor.getDistance(DistanceUnit.CM) > (flatWall + 10) && opModeIsActive() && (maxDepth - 20) < robot.distanceSensor.getDistance(DistanceUnit.CM) && robot.colorSensor.alpha() < (robot.maxBrightness - 200)) {
                if (robot.distanceSensor.getDistance(DistanceUnit.CM) > maxDepth && robot.distanceSensor.getDistance(DistanceUnit.CM) > (flatWall + 35)) {
                    maxDepth = robot.distanceSensor.getDistance(DistanceUnit.CM);
                }
                    robot.imu.resetYaw();
                    robot.motorLeft.setPower(power * .58);
                    robot.motorRight.setPower(power * -.58);
                    robot.motorAux.setPower(.1 * getHeading());
                    System.out.println("Distance Traveled" + robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition()));
                    System.out.println("Block Distance" + robot.distanceSensor.getDistance(DistanceUnit.CM));
//                    if (robot.distanceSensor.getDistance(DistanceUnit.CM) > maxDepth) {
//                        maxDepth = robot.distanceSensor.getDistance(DistanceUnit.CM);
//                    }
                    currentSpotLength = robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition()) - startDistance;
                }
            if (minimumSpotLength > currentSpotLength && currentSpotLength>=35){
                    minimumSpotLength = currentSpotLength;
                    System.out.println("minimumspot"+minimumSpotLength);
                    distanceToGoBack = robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition());
                System.out.println("goback"+distanceToGoBack);
                }

            }


//            double parkingWidth = robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition()) - startDistance;
        robot.startMove(0,0,0,0);
//            System.out.println("parking width"+parkingWidth);
        System.out.println("isittrue"+(minimumSpotLength>=35));
        System.out.println("isittrue"+((maxDepth-flatWall)>=35));
        System.out.println("maxdepth"+maxDepth);
        if (minimumSpotLength>=35 && ((maxDepth-flatWall)>=35)) {
            System.out.println("imhere");
            sleep(2000);
            double driveBack = (robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition())-distanceToGoBack)+20;
            robot.resetDriveEncoders();
            driveIMU(driveBack,-.1);
            sleep(2000);
            while (robot.distanceSensor.getDistance(DistanceUnit.CM)>25 && opModeIsActive()) {
                System.out.println("Block DistanceParking"+ robot.distanceSensor.getDistance(DistanceUnit.CM));
                robot.startMove(0,.1,0,0);
            }
            didWePark=true;
        }

        robot.startMove(0,0,0,0);
        System.out.println("didwepark"+didWePark);
        if (!didWePark) {
            double distance = robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition());
            robot.resetDriveEncoders();
            driveIMU(distance, -.3);
            robot.startMove(0,0,0,0);
            sleep(2000);
            turnIMU(-90,.3);
            sleep(2000);
            while (robot.colorSensor.alpha()<(robot.maxBrightness-200) && opModeIsActive()) {
                robot.imu.resetYaw();
                robot.motorLeft.setPower(-power*.58);
                robot.motorRight.setPower(-power*-.58);
                robot.motorAux.setPower(.1*getHeading());
            }

        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        lookingForParking(.2);
    }

}
