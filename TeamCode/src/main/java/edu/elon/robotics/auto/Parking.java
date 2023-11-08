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
        sleep(2000);
        robot.resetDriveEncoders();
        double flatWall = robot.distanceSensor.getDistance(DistanceUnit.CM);
        System.out.println("Flat wall"+ flatWall);
        boolean didWePark = false;

        while (robot.colorSensor.alpha()<(robot.maxBrightness-200) && opModeIsActive() && !didWePark) {
            while (robot.distanceSensor.getDistance(DistanceUnit.CM)<(flatWall+5) && opModeIsActive()) {
                robot.imu.resetYaw();
                robot.motorLeft.setPower(power*.58);
                robot.motorRight.setPower(power*-.58);
                robot.motorAux.setPower(.1*getHeading());
                System.out.println("Distance Traveled"+ robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition()));
                System.out.println("Block Distance"+ robot.distanceSensor.getDistance(DistanceUnit.CM));
            }
            double startDistance = robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition());
            System.out.println("Start distance: "+ startDistance);
            double maxDepth = 0;
//            double minDepth = 1000000;
            while (robot.distanceSensor.getDistance(DistanceUnit.CM)>=(flatWall+5) && opModeIsActive() && (maxDepth-20)<robot.distanceSensor.getDistance(DistanceUnit.CM)) {
                robot.imu.resetYaw();
                robot.motorLeft.setPower(power*.58);
                robot.motorRight.setPower(power*-.58);
                robot.motorAux.setPower(.1*getHeading());
                System.out.println("Distance Traveled"+ robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition()));
                System.out.println("Block Distance"+ robot.distanceSensor.getDistance(DistanceUnit.CM));
                if (robot.distanceSensor.getDistance(DistanceUnit.CM) > maxDepth) {
                    maxDepth = robot.distanceSensor.getDistance(DistanceUnit.CM);
                }
//                if (robot.distanceSensor.getDistance(DistanceUnit.CM) < minDepth) {
//                    minDepth = robot.distanceSensor.getDistance(DistanceUnit.CM);
//                }
//                if ((maxDepth-20)>robot.distanceSensor.getDistance(DistanceUnit.CM)) {
//                    break;
//                }
            }
            double parkingWidth = robot.convertTicksToDistance(robot.motorLeft.getCurrentPosition()) - startDistance;
            robot.startMove(0,0,0,0);
            System.out.println("parking width"+parkingWidth);

            if (parkingWidth>=35 && (maxDepth-flatWall>=35)) {
                sleep(2000);
                driveDistance(20,0,-.1);
                sleep(2000);
                while (robot.distanceSensor.getDistance(DistanceUnit.CM)>25 && opModeIsActive()) {
                    System.out.println("Block DistanceParking"+ robot.distanceSensor.getDistance(DistanceUnit.CM));
                    robot.startMove(0,.1,0,0);
                }
                didWePark=true;
            }
            robot.startMove(0,0,0,0);
            sleep(2000);
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
