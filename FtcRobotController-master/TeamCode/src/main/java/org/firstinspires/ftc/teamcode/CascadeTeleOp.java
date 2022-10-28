package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@TeleOp(name="Super Cool OP Mode \uD83D\uDE0E", group="Cool OP modes")
public class CascadeTeleOp extends LinearOpMode {

    //private DcMotorEx coreHex;
    private DcMotorEx motor;
    private DcMotorEx extend;
    DistanceSensor colorSensor;
    int error = 0;
    int pos = 0;
    double distance;
    int ticks = 10;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initialize();

        motor.setPower(0.2);
        sleep(350);
        motor.setPower(0);
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            distance = colorSensor.getDistance(DistanceUnit.CM);
            /*
            coreHex.setPower(gamepad1.left_trigger);
            coreHex.setPower(-gamepad1.right_trigger * 0.6);
             */
            extend.setPower(-gamepad1.left_trigger * 0.2);
            extend.setPower(gamepad1.right_trigger * 0.3);
            if (gamepad1.a) {
                motor.setPower(1);
                telemetry.addLine("Doing the thing, yeah yeah doing the thing");
                telemetry.update();
                long start = System.currentTimeMillis();
                long end = start + 1900;
                while (System.currentTimeMillis() < end){
                    if (gamepad1.x) {
                        motor.setPower(0);
                        break;
                    }
                    telemetry.addLine("Doing the thing, yeah yeah doing the thing");
                    telemetry.addLine("Press X if things go Wacko!!!!!!!!!!!");
                    telemetry.update();
                }
                motor.setPower(0);
                telemetry.addLine("DONE!");
                telemetry.update();
            }

            if (gamepad1.b) {
                if (distance > 4.2) {
                    motor.setPower(-1);
                    distance = colorSensor.getDistance(DistanceUnit.CM);
                    while (opModeIsActive() && distance > 4.2) {
                        if (gamepad1.x) {
                            motor.setPower(0);
                            break;
                        }
                        distance = colorSensor.getDistance(DistanceUnit.CM);
                        telemetry.addLine("Doing the thing, yeah yeah doing the thing");
                        telemetry.addData("Tick: ", motor.getCurrentPosition());
                        telemetry.update();

                    }
                    motor.setPower(0);
                }
            }
        }
    }
    private void initialize(){
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        extend = hardwareMap.get(DcMotorEx.class, "extend");
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        /*
        coreHex = hardwareMap.get(DcMotorEx.class, "claw");
        coreHex.setDirection(DcMotorSimple.Direction.FORWARD);
         */

    }
}
