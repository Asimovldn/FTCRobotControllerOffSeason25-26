package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Odometry.Encoder;

@TeleOp
public class TesteOdometria extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        DcMotorEx odometryMotor = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        Encoder odometryEncoder = new Encoder(odometryMotor);
        waitForStart();

        int displacement = 0;
        int lastPos = 0;

        double pos = 0;
        while (opModeIsActive()) {
            double Ticks2Cm = (Encoder.WHEEL_DIAMETER * 3.1415) / Encoder.ENCODER_TICKS;
            telemetry.addData("EncoderPos", odometryEncoder.getPosition());
            telemetry.addData("EncoderVel", odometryEncoder.getAngularVelocity(AngleUnit.DEGREES));

            int currentPos = odometryEncoder.getPosition();
            displacement = currentPos - lastPos;

            pos += displacement * Ticks2Cm;

            lastPos = currentPos;

            telemetry.addData("pos", pos);
            telemetry.update();
        }
    }
}
