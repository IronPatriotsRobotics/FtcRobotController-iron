/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;



@TeleOp(name="Main Drive", group="Robot")
//@Disabled
public class GoBildaRi3D2425 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftFrontDrive   = null; //the left drivetrain motor
    public DcMotor  rightFrontDrive  = null; //the right drivetrain motor
    public DcMotor  leftBackDrive    = null;
    public DcMotor  rightBackDrive   = null;
    public DcMotor  armMotor         = null; //the arm motor
    public DcMotor  liftMotor        = null; //
    public DcMotor  hangMotor        = null;
    public CRServo  intake           = null; //the active intake servo
    public Servo    wrist            = null; //the wrist servo

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 4 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 100 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 140 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 5  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.03;
    final double WRIST_FOLDED_OUT  = 0.32;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_COLLAPSED;

    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double armLiftComp = 0;


    @Override
    public void runOpMode() {
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double left;
        double right;
        double forward;
        double rotate;
        double max;


        /* Define and Initialize Motors */
        leftFrontDrive  = hardwareMap.dcMotor.get("frontLeft");
        leftBackDrive   = hardwareMap.dcMotor.get("backLeft");
        rightFrontDrive = hardwareMap.dcMotor.get("frontRight");
        rightBackDrive  = hardwareMap.dcMotor.get("backRight");
        liftMotor       = hardwareMap.dcMotor.get("viper");
        armMotor        = hardwareMap.dcMotor.get("lift_arm"); //the arm motor
        //hangMotor       = hardwareMap.dcMotor.get("viper");


       /*
       we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
       drive motors to go forward.
        */

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_OUT);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive())

        {  double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (-rotY + rotX + rx) / denominator;
            double backLeftPower = (-rotY - rotX + rx) / denominator;
            double frontRightPower = (-rotY - rotX - rx) / denominator;
            double backRightPower = (-rotY + rotX - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);


            /* Here we handle the three buttons that have direct control of the intake speed.
            These control the continuous rotation servo that pulls elements into the robot,
            If the user presses A, it sets the intake power to the final variable that
            holds the speed we want to collect at.
            If the user presses X, it sets the servo to Off.
            And if the user presses B it reveres the servo to spit out the element.*/

            /* TECH TIP: If Else statement:
            We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in case
            multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
            at the same time. "a" will win over and the intake will turn on. If we just had
            three if statements, then it will set the intake servo's power to multiple speeds in
            one cycle. Which can cause strange behavior. */

            if (gamepad1.left_bumper) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad1.right_bumper) {
                intake.setPower(INTAKE_DEPOSIT);
            }
            else if (gamepad1.y) {
                intake.setPower(INTAKE_OFF);
            }




            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));



            if(gamepad2.a){
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                liftPosition = LIFT_COLLAPSED;
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);
            }
            else if(gamepad2.y){
                intake.setPower(INTAKE_OFF);
            }

            else if (gamepad2.b){
                    /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
                armPosition = ARM_CLEAR_BARRIER;
            }

            else if (gamepad2.x){
                /* This is the correct height to score the sample in the HIGH BASKET */
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
                //liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
                wrist.setPosition(WRIST_FOLDED_OUT);
            }

            else if (gamepad2.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                liftPosition = LIFT_COLLAPSED;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad2.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                liftPosition = LIFT_COLLAPSED;
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad2.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad2.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            if (armPosition < 45 * ARM_TICKS_PER_DEGREE){
                armLiftComp = (0.25568 * liftPosition);
            }
            else{
                armLiftComp = 0;
            }


            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if (gamepad2.right_bumper){
                liftPosition += 2800 * cycletime;
            }
            else if (gamepad2.left_bumper){
                liftPosition -= 2800 * cycletime;
            }
            /*here we check to see if the lift is trying to go higher than the maximum extension.
             *if it is, we set the variable to the max.
             */
            if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET){
                liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }
            //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
            if (liftPosition < 0){
                liftPosition = 0;
            }

            liftMotor.setTargetPosition((int) (liftPosition));

            ((DcMotorEx) liftMotor).setVelocity(2100);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }


            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("arm Target Position: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.addData("lift variable", liftPosition);
            telemetry.addData("Lift Target Position",liftMotor.getTargetPosition());
            telemetry.addData("lift current position", liftMotor.getCurrentPosition());
            telemetry.addData("liftMotor Current:",((DcMotorEx) liftMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.update();



        }
    }
}