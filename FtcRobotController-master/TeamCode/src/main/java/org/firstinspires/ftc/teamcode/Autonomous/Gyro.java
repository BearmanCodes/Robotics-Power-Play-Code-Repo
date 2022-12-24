package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled
@Autonomous(name="Gyro", group="TESTS")
public class Gyro extends LinearOpMode {
    private final DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
    private final DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
    private final DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
    private final DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
    int frontRightTarget;
    int frontLeftTarget;
    int backRightTarget;
    int backLeftTarget;
    double driveSpeed;
    double turnSpeed;
    double targetHeading;
    double frontLeftSpeed;
    double frontRightSpeed;
    double backLeftSpeed;
    double backRightSpeed;
    double robotHeading = 0;
    double headingOffset = 0;
    double headingError = 0;
    DcMotorEx[] allMotors = {frontLeft, frontRight, backLeft, backRight};
    BNO055IMU imu;
    static final double P_DRIVE_GAIN = 0.03;

    static final double TicksPerRev = 560;
    static final double WheelInches = (75 / 25.4);
    static final double TicksPerIn = TicksPerRev / (WheelInches * Math.PI);

    @Override
    public void runOpMode(){
        initialize();

        waitForStart();

        allMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        straight(0.4, 48.0, 0);
    }

    public void straight(double maxDriveSpeed,
                         double distance,
                         double heading){
        if (opModeIsActive()){
            frontRightTarget = frontRight.getCurrentPosition() + (int) (distance * TicksPerIn);
            frontLeftTarget = frontLeft.getCurrentPosition() + (int) (distance * TicksPerIn);
            backRightTarget = backRight.getCurrentPosition() + (int) (distance * TicksPerIn);
            backLeftTarget = backLeft.getCurrentPosition() + (int) (distance * TicksPerIn);

            setTargetPosition(frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
            allMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            GoGoGo(maxDriveSpeed, 0);

            while(opModeIsActive() && busy()){
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                GoGoGo(driveSpeed, turnSpeed);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            GoGoGo(0, 0);
            allMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }


    private boolean busy(){
        return frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
    }
    private void allMotorMode(DcMotor.RunMode mode){
        for (DcMotorEx motor: allMotors){
            motor.setMode(mode);
        }
    }

    private void allMotorPower(double power){
        for (DcMotorEx motor : allMotors){
            motor.setPower(power);
        }
    }

    private void setMotorPower(double frontLeftPower, double frontRightPower, double backLeftPower,
                               double backRightPower){
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void setTargetPosition(int frontLeftPos, int frontRightPos,
                                   int backLeftPos, int backRightPos){
        frontLeft.setTargetPosition(frontLeftPos);
        frontRight.setTargetPosition(frontRightPos);
        backLeft.setTargetPosition(backLeftPos);
        backRight.setTargetPosition(backRightPos);
    }

    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading(){
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    private void initialize() {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        allMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void GoGoGo(double drive, double turn){
        driveSpeed = drive;
        turnSpeed = turn;

        frontLeftSpeed = drive + turn;
        frontRightSpeed = drive + turn;
        backLeftSpeed = drive + turn;
        backRightSpeed = drive + turn;

        double maxFront = Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed));
        double maxBack = Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed));
        if (maxFront > 1.0 || maxBack > 1.0){
            frontLeftSpeed /= maxFront;
            frontRightSpeed /= maxFront;
            backLeftSpeed /= maxBack;
            backRightSpeed /= maxBack;
        }
        setMotorPower(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

}