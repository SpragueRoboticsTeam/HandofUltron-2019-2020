package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*class RLegos {
    public float HSV[] = {0f, 0f, 0f};
    public boolean isThere = true;
    public float distance = 0;
} */

@Autonomous
public class RedRight extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1680;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1.0;
    static final double     TURN_SPEED              = 0.5;



    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor front = null;
    private DcMotor back = null;
    private DcMotor claw = null;
    private DcMotor lift = null;
    private ColorSensor frontCS = null;
    private DistanceSensor frontDS = null;
    private ColorSensor platCS = null;
    private DistanceSensor platDS = null;
    private Servo leftServo = null;
    private Servo rightServo = null;

    public float HSVF[] = {0f, 0f, 0f};
    public float HSVD[] = {0f, 0f, 0f};

    final double SCALE_FACTOR = 255;

    //private Legos[] legoArray = new Legos[5];

    //thresh hold red<50 is skystone

    public void runOpMode() {

        left = hardwareMap.get(DcMotor.class, "L");
        right = hardwareMap.get(DcMotor.class, "R");
        front = hardwareMap.get(DcMotor.class, "F");
        back = hardwareMap.get(DcMotor.class, "B");

        leftServo = hardwareMap.get(Servo.class, "LS");
        rightServo = hardwareMap.get(Servo.class, "RS");
        claw = hardwareMap.get(DcMotor.class, "C");
        lift = hardwareMap.get(DcMotor.class, "S");

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        front.setDirection(DcMotor.Direction.FORWARD);
        back.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        leftServo.setPosition(0.0);
        rightServo.setPosition(1.0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                left.getCurrentPosition(),
                right.getCurrentPosition(),
                front.getCurrentPosition(),
                back.getCurrentPosition());
        telemetry.update();

        waitForStart();
        encoderDrive(DRIVE_SPEED,   0, 0, -21, -21); // Right 20 inches
        encoderDrive(DRIVE_SPEED,   43, 45, 0, 0); // Forward towards platform
        claw.setPower(1); // Initializes claw
        leftServo.setPosition(1.0); // Puts servos down on platform
        rightServo.setPosition(0.0);
        sleep(500); // Gives time to allow servos to move
        claw.setPower(0); // Stops claw so it isn't running forever
        encoderDrive(DRIVE_SPEED,    35, -35, -45, 45); // S4: Backwards 45 inches at full speed
        leftServo.setPosition(0); // Releases platform
        rightServo.setPosition(1);
        sleep(200); // Gives servos time


        encoderDrive(DRIVE_SPEED, -75, -75, 0, 0); // S6: Drive 80 inches to the right for team bridge
        encoderTurn(DRIVE_SPEED, -76); // S8: Rotate (what's supposed to be) 90 degrees clockwise to face lego
        encoderDrive(DRIVE_SPEED, 22, 22, 0, 0); // S7: Drive 22 inches forward towards legos
        liftMove(2000); // Moves lift so it doesn't collide with lego on following step
        encoderDrive(DRIVE_SPEED, 0, 0, 8, 8); // Moves  8 inches forward (technically to the left because of orientation change) to get to legos
        ClawDown(false); // Captures lego with claw
        liftMove(1200); // Lifts lego up so it doesn't drag on the ground


        encoderTurn(DRIVE_SPEED, 105); // Turns towards alliance bridge
        encoderDrive(DRIVE_SPEED, 0, 0, 39, 39); // Drives 39 inches forward past alliance bridge
        ClawUp(); // Releases lego
        encoderDrive(DRIVE_SPEED, 0, 0, -20, -20); // Drives back onto Alliance bridge
        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double frontInches, double backInches) {
        int newLeftTarget;
        int newRightTarget;
        int newFrontTarget;
        int newBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackTarget = back.getCurrentPosition() + (int)(backInches * COUNTS_PER_INCH);
            newFrontTarget = front.getCurrentPosition() + (int)(frontInches * COUNTS_PER_INCH);
            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);
            front.setTargetPosition(newFrontTarget);
            back.setTargetPosition(newBackTarget);

            // Turn On RUN_TO_POSITION
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.

            left.setPower(Math.abs(speed));
            right.setPower(Math.abs(speed));
            front.setPower(Math.abs(speed));
            back.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&

                    (left.isBusy() || right.isBusy() || front.isBusy() || back.isBusy())) {

                // Display it for the driver.
                idle();

            }

            // Stop all motion;
            left.setPower(0);
            right.setPower(0);
            front.setPower(0);
            back.setPower(0);

            // Turn off RUN_TO_POSITION
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void liftMove(int idk){

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            lift.setTargetPosition(idk);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (lift.isBusy())) {

                idle();

            }

            // Stop all motion;
            lift.setPower(0);


            // Turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderTurn (  double speed, double angle) {
        telemetry.addData("Degrees: ", angle);

        front.setDirection(DcMotor.Direction.REVERSE);
        back.setDirection(DcMotor.Direction.FORWARD);

        double fullRotation = 1680 * 3;
        double turnFract = 360 / Math.abs(angle);
        telemetry.addData("turn fraction: ", turnFract);

        int newLeftTarget = 0, newRightTarget = 0;

        double countNum = fullRotation / turnFract;
        telemetry.addData("encoder count", countNum);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (angle > 0) {
            newLeftTarget = left.getCurrentPosition() - (int) countNum;
            newRightTarget = right.getCurrentPosition() + (int) countNum;
            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);

            newLeftTarget = back.getCurrentPosition() - (int) countNum;
            newRightTarget = front.getCurrentPosition() + (int) countNum;
            back.setTargetPosition(newLeftTarget);
            front.setTargetPosition(newRightTarget);
        }

        if (angle < 0) {
            newLeftTarget = left.getCurrentPosition() + (int) countNum;
            newRightTarget = right.getCurrentPosition() - (int) countNum;
            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);

            newLeftTarget = back.getCurrentPosition() + (int) countNum;
            newRightTarget = front.getCurrentPosition() - (int) countNum;
            back.setTargetPosition(newLeftTarget);
            front.setTargetPosition(newRightTarget);
        }


        left.setPower(Math.abs(speed));
        right.setPower(Math.abs(speed));
        front.setPower(Math.abs(speed));
        back.setPower(Math.abs(speed));

        front.setDirection(DcMotor.Direction.FORWARD);
        back.setDirection(DcMotor.Direction.REVERSE);

        while (opModeIsActive() && (left.isBusy() && right.isBusy())) {
            idle();

        }

        // Turn off RUN_TO_POSITION
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop all motion;
        left.setPower(0);
        right.setPower(0);
        front.setPower(0);
        back.setPower(0);


        telemetry.addData("turn", "complete");
        telemetry.update();
        sleep(100);

    }

    public void ClawDown(boolean beginning) {
        if(beginning) {
            claw.setPower(1);
            sleep(350);
            claw.setPower(-1);
            sleep(250);
            claw.setPower(-0.1);
        } else {
            claw.setPower(-1);
            sleep(250);
            claw.setPower(-0.1);
        }
    }

    public void ClawUp() {
        claw.setPower(1);
        sleep(250);
        claw.setPower(0.1);
    }
}
