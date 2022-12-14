package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Requiesce", group="Cool OP modes")
public class Requiesce extends LinearOpMode {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx arm;
    private final ElapsedTime time = new ElapsedTime();
    DistanceSensor colorSensor;
    double cascadePower;
    double distance;
    double extendPower;
    Gamepad currentGamepad = new Gamepad(); //These 2 are for an edge detector in case I need one later.
    Gamepad previousGamepad = new Gamepad();
    double backRightPower;
    double frontRightPower;
    double backLeftPower;
    double frontLeftPower;
    double Vertical;
    double Horizontal;
    double Pivot;
    public CRServo crServo;


    double reducer = 0.65;
    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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
        colorSensor = hardwareMap.get(DistanceSensor.class, "colorSensor"); //Assign the color sensor.
        crServo = hardwareMap.get(CRServo.class, "servo");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD); //Reverses it's direction so that it goes the right way.
        crServo.setDirection(DcMotorSimple.Direction.REVERSE);

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
        long end = start + 100;
        while (System.currentTimeMillis() < end){
            telemetry.addLine("Tightening");
            telemetry.update();
        }
        arm.setPower(0);
        telemetry.addLine("Let's up Requiesce"); //It passed the fire test. WOOOH!
        telemetry.update();
    }

    private void ArmGo(){
        distance = colorSensor.getDistance(DistanceUnit.CM); //Update the color sensor distance right off the bat.
        cascadePower = -gamepad2.left_trigger + gamepad2.right_trigger;
        if(distance > 4.2 || cascadePower > 0){
            arm.setPower(cascadePower);
        }else {
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void ClawGo(){
       if (gamepad2.right_bumper){
           crServo.setPower(1);
       }
       if (gamepad2.left_bumper){
           crServo.setPower(-1);
       }
       if (!gamepad2.right_bumper && !gamepad2.left_bumper){
           crServo.setPower(0);
       }
    }
}
