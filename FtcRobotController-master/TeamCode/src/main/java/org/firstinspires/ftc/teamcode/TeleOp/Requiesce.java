package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Requiesce", group="Cool OP modes")
public class Requiesce extends LinearOpMode {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    double Vertical;
    double Horizontal;
    double Pivot;
    double frontLeftPower;
    double frontRightPower;
    double backRightPower;
    double backLeftPower;
    Servo lClaw;
    Servo rClaw;
    boolean clawToggle = false;
    double cascadePower;
    private DcMotorEx arm;
    private final ElapsedTime time = new ElapsedTime();
    Gamepad currentGamepad = new Gamepad(); //These 2 are for an edge detector in case I need one later.
    Gamepad previousGamepad = new Gamepad();

    double reducer = 0.65;
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (rClaw.getPosition() >= 0.25 && lClaw.getPosition() < 0.29){
                lClaw.setPosition(0.29);
            }
            DriveGo();
            ArmGo();
            ClawGo();
        }
    }

    private void initialize(){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        arm = hardwareMap.get(DcMotorEx.class, "motor"); //Assign the cascading kit motor.
        lClaw = hardwareMap.get(Servo.class, "lclaw");
        rClaw = hardwareMap.get(Servo.class, "rclaw");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD); //Reverses it's direction so that it goes the right way.
        rClaw.setDirection(Servo.Direction.REVERSE);
        lClaw.setPosition(0.15);
        rClaw.setPosition(0);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //STOP once the power's 0, prevents slipping.


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //These lines tighten the string just to make sure it's in the spool like it should be.
        arm.setPower(0.2);
        long start = System.currentTimeMillis();
        long end = start + 250;
        while (System.currentTimeMillis() < end){
            telemetry.addLine("Tightening");
            telemetry.update();
        }
        arm.setPower(0);
        telemetry.addLine("Let's up Requiesce"); //It passed the fire test. WOOOH!
        telemetry.update();
    }

    private void DriveGo(){
        Vertical = gamepad1.left_stick_y;
        Horizontal = gamepad1.left_stick_x * 1.2;
        Pivot = gamepad1.right_stick_x;

        frontLeftPower = (-Pivot + (Vertical - Horizontal)) * reducer;
        frontRightPower = (Pivot + Vertical + Horizontal) * reducer;
        backRightPower = (Pivot + (Vertical - Horizontal)) * reducer;
        backLeftPower = (-Pivot + Vertical + Horizontal) * reducer;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void ArmGo(){
        cascadePower = -gamepad2.left_trigger + gamepad2.right_trigger;
        arm.setPower(cascadePower);
    }

    private void ClawGo(){
        //Left claw, 0.1 opens it up, 0.3 closes it enough, 0.35/0.4 to really clamp.
        //Right claw, 0.0 opens it up, 0.2 closes it just a bit, 0.25/0.3 to really close.
        try {
            edgeDetector();
        } catch (RobotCoreException e){
            telemetry.addLine("WHAT HAPPEN");
            telemetry.update();
        }
        if (currentGamepad.a && !previousGamepad.a){ //Change to A once I get back
            clawToggle = !clawToggle;
        }
        if (clawToggle){
            lClaw.setPosition(0.29);
            rClaw.setPosition(0.28);
        }
        else {
            lClaw.setPosition(0.15);
            rClaw.setPosition(0);
        }
    }

    public void edgeDetector() throws RobotCoreException {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad2);
    }
}
