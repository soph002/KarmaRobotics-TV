package edu.elon.robotics;

/*
 * Defines the robot hardware and implements a few
 * fundamental methods.
 * adb connect 192.168.43.1
 */

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    // drive motors
    public DcMotor motorLeft;
    public DcMotor motorRight;
    public DcMotor motorAux;

    // robot constants
    public final double TICKS_PER_ROTATION = 537.6;
    public final double WHEEL_CIRCUMFERENCE = 34; // in cm
    public final double TICKS_PER_CM = TICKS_PER_ROTATION/WHEEL_CIRCUMFERENCE;
    public final double TURNING_DIAMETER=35.8; // in cm
    public final double FULL_CIRCLE=360.0;
    // 35 cm FROM CIRCLE ON GROUND
    // s= 29.5 cm
    // r = 17.5 cm

    //control hub IMU
    public IMU imu;
    // roll is left/right roll
    // yaw is turning
    // pitch is front to back, ramp style

    //sensors
    public DigitalChannel touchSensor;
    public ColorSensor colorSensor;

    public int maxBrightness;

    public int minBrightness;
    public DistanceSensor distanceSensor;

    // all the variable for the arm
    public DcMotor motorArm;

    public Servo servoGripper;

    public Servo servoWrist;

    public ColorSensor colorArm;

    // constants for controlling the arm
    public final double ARM_INIT_POWER = -0.15; // init speed of the motor
    public final double ARM_POWER_UP = 0.3;     // up speed of the motor
    public final double ARM_POWER_DOWN = -0.2;  // down speed of the motor
    public final int ARM_MAX_HEIGHT = 1200;     // encoder ticks of upper limit

    // constants for controlling the wrist
    public static final double WRIST_START_POS = 0.7; // parallel to ground
    public static final double WRIST_PICKUP_POS = 0.65; // parallel to ground
    public final double WRIST_FULLY_DOWN = 0.1;        // minimum position
    public final double WRIST_FULLY_UP = 0.9;          // maximum position
    public final double WRIST_INCREMENT = 0.05;        // delta value

    // constants for controlling the gripper
    public static final double GRIPPER_FULLY_OPEN = 0.9; // gripper open
    public final double GRIPPER_FULLY_CLOSED = 0.45;     // gripper closed
    public final double GRIPPER_INCREMENT = 0.05;        // delta value


    public RobotHardware(HardwareMap hardwareMap) {
        /*
         * This code provides an object to control the physical
         * motor referenced by the configuration string.  The
         * positive direction of rotation is established. Finally,
         * the motor is directed to forcefully stop when no power
         * is applied.
         */

        // define the drive motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorAux = hardwareMap.dcMotor.get("motorAux");
        motorAux.setDirection(DcMotorSimple.Direction.FORWARD);
        motorAux.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorArm.setDirection(DcMotorSimple.Direction.REVERSE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoGripper = hardwareMap.get(Servo.class,"servoGripper");
        // reset the servos (temporary for now)
        servoGripper.setPosition(0.5);
        servoWrist = hardwareMap.get(Servo.class,"servoWrist");
        servoWrist.setPosition(0.5);

        colorArm = hardwareMap.get(ColorSensor.class, "colorArm");
        colorArm.enableLed(true);

        // reset the drive encoders to zero
        resetDriveEncoders();

        /*
        The next three lines define Hub orientation.
        Our hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
        */

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        // Let's define the direction the robot starts pointing at as 0 degrees (of Yaw)
        imu.resetYaw();


    }

    public int convertDistanceToTicks(double cm){
        return (int) (Math.abs(cm) * TICKS_PER_CM);
    }

    public double convertTicksToDistance(double ticks){
        return Math.abs(ticks) / TICKS_PER_CM;
    }


    public int convertDegreesToTicks(double degrees){
        double arcLength = (degrees/FULL_CIRCLE) * Math.PI * TURNING_DIAMETER;

        return (int) (arcLength * TICKS_PER_CM);
    }

    public void resetDriveEncoders() {
        /*
         * This code resets the encoder values back to 0 for
         * each of the three drive motors.
         */
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorAux.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorAux.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void startMove(double drive, double strafe, double turn, double modifier) {
        /*
         * How much power should we apply to the left,
         * right, and aux motor?
         *
         * If all 3 motors apply the same power in the
         * same direction, the robot will turn in place.
         */
        double powerLeft= 0.58 * drive + 1.0 * strafe / 3.0 + turn / 3.0;
        double powerRight= -0.58 * drive + 1.0 * strafe / 3.0 + turn / 3.0;
        double powerAux= -2.0 * strafe / 3.0 + turn / 3.0;

        double scale = Math.max(Math.max(Math.max(1.0, Math.abs(powerLeft)), Math.abs(powerRight)), Math.abs(powerAux));

        powerLeft=powerLeft/scale;
        powerRight=powerRight/scale;
        powerAux=powerAux/scale;
        /*
         * Apply the power to the motors.
         */
        motorLeft.setPower(powerLeft);
        motorRight.setPower(powerRight);
        motorAux.setPower(powerAux);

    }


}
