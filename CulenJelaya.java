package org.firstinspires.ftc.teamcode.Teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="CulenJelaya", group="Linear Opmode")
public class CulenJelaya extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor front = null;
    private DcMotor back = null;
    private ColorSensor platformCS = null;
    private ColorSensor frontCS = null;
    private DistanceSensor platformDS = null;
    private DistanceSensor frontDS = null;
    private Servo leftServo = null;
    private Servo rightServo = null;

    public float HSVF[] = {0f, 0f, 0f};
    public float HSVD[] = {0f, 0f, 0f};

    final double SCALE_FACTOR = 255;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left = hardwareMap.get(DcMotor.class, "L");
        right = hardwareMap.get(DcMotor.class, "R");
        front = hardwareMap.get(DcMotor.class, "F");
        back = hardwareMap.get(DcMotor.class, "B");
        platformCS = hardwareMap.get(ColorSensor.class, "PCS");
        frontCS = hardwareMap.get(ColorSensor.class, "DCS");
        frontDS = hardwareMap.get(DistanceSensor.class, "FDS");
        platformDS = hardwareMap.get(DistanceSensor.class, "PCS");
        leftServo = hardwareMap.get(Servo.class, "LS");
        rightServo = hardwareMap.get(Servo.class, "RS");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);
        front.setDirection(DcMotor.Direction.FORWARD);
        back.setDirection(DcMotor.Direction.REVERSE);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftServo.setPosition(0);
        rightServo.setPosition(1);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            Color.RGBToHSV((int) (frontCS.red() * SCALE_FACTOR), (int) (frontCS.green() * SCALE_FACTOR), (int) (frontCS.blue() * SCALE_FACTOR), HSVF);
            
            if(gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0) {
                left.setPower(gamepad1.left_stick_y);
                right.setPower(gamepad1.left_stick_y);
                front.setPower(-gamepad1.left_stick_x);
                back.setPower(-gamepad1.left_stick_x);
            }
            else if(gamepad1.right_stick_x != 0) {
                left.setPower(gamepad1.right_stick_x);
                right.setPower(-gamepad1.right_stick_x);
                front.setPower(-gamepad1.right_stick_x);
                back.setPower(gamepad1.right_stick_x);
            } else {
                left.setPower(0);
                right.setPower(0);
                front.setPower(0);
                back.setPower(0);
            }
            
            if(gamepad1.a) {
                leftServo.setPosition(1);
                rightServo.setPosition(0);
            }
            else if(gamepad1.y) {
                leftServo.setPosition(0);
                rightServo.setPosition(1);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Distance", frontDS.getDistance(DistanceUnit.INCH));
            telemetry.addData("Color", frontCS.blue());
            telemetry.update();
        }
    }
}
