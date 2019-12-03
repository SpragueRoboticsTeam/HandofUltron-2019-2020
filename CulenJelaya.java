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


@TeleOp
public class CulenJelaya_OBJ extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor front = null;
    private DcMotor back = null;
    private DcMotor claw = null;
    private DcMotor lift = null;
   // private ColorSensor platformCS = null;
    private ColorSensor frontCS = null;
   // private DistanceSensor platformDS = null;
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

        left = hardwareMap.get(DcMotor.class, "B");
        right = hardwareMap.get(DcMotor.class, "F");
        front = hardwareMap.get(DcMotor.class, "L");
        back = hardwareMap.get(DcMotor.class, "R");
        claw = hardwareMap.get(DcMotor.class, "C");
        lift = hardwareMap.get(DcMotor.class, "S");
        //FRBl
        //RBLF

        //platformCS = hardwareMap.get(ColorSensor.class, "PCS");
        frontCS = hardwareMap.get(ColorSensor.class, "FCS");
        frontDS = hardwareMap.get(DistanceSensor.class, "FDS");
       // platformDS = hardwareMap.get(DistanceSensor.class, "PCS");
        leftServo = hardwareMap.get(Servo.class, "LS");
        rightServo = hardwareMap.get(Servo.class, "RS");

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        front.setDirection(DcMotor.Direction.REVERSE);
        back.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setTargetPosition(0);
        //lift.setPower(.5);
        
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
                left.setPower(-gamepad1.right_stick_x);
                right.setPower(gamepad1.right_stick_x);
                front.setPower(gamepad1.right_stick_x);
                back.setPower(-gamepad1.right_stick_x);
            }
            else if(gamepad2.left_bumper) {
                leftServo.setPosition(1);
                rightServo.setPosition(0);
            }
            else if(gamepad2.right_bumper) {
                leftServo.setPosition(0);
                rightServo.setPosition(1);
            }
            else{
                left.setPower(0);
                right.setPower(0);
                front.setPower(0);
                back.setPower(0);
            }

            if(gamepad2.left_stick_y < 0 && lift.getCurrentPosition() < 27000){//
                lift.setPower(1);
            }
            else if(gamepad2.left_stick_y > 0 && lift.getCurrentPosition() > -1){//
                lift.setPower(-1);
            }
            else{
                lift.setPower(0);
            }

            if(gamepad2.y){
                claw.setPower(0.35);
            }
            else if(gamepad2.a){
                claw.setPower(-0.35);
            }
            else{
                claw.setPower(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData( "lift: " , lift.getCurrentPosition());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Distance", frontDS.getDistance(DistanceUnit.INCH));
            telemetry.addData("Color", frontCS.blue());
            telemetry.update();
        }
    }
}
