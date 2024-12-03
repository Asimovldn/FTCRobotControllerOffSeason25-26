package org.firstinspires.ftc.teamcode.Autonomo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auxiliar.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Localizer;

@Autonomous
public class StraightTest extends LinearOpMode
{
    DcMotor fr_esq;
    DcMotor fr_dir;
    DcMotor ba_esq;
    DcMotor ba_dir;

    Localizer localizer;

    @Override
    public void runOpMode() throws InterruptedException
    {

        fr_esq = hardwareMap.get(DcMotor.class,"left_front_drive");
        fr_dir = hardwareMap.get(DcMotor.class,"right_front_drive");
        ba_esq = hardwareMap.get(DcMotor.class,"left_back_drive");
        ba_dir = hardwareMap.get(DcMotor.class,"right_back_drive");

        ba_dir.setDirection(DcMotorSimple.Direction.REVERSE);
        fr_dir.setDirection(DcMotorSimple.Direction.REVERSE);

        localizer = new Localizer(hardwareMap);

        waitForStart();

        fr_esq.setPower(0.5);
        fr_dir.setPower(0.5);
        ba_esq.setPower(0.5);
        ba_dir.setPower(0.5);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        timer.reset();

        while (timer.time() < 2)
        {
            localizer.update();

            Vector2D pos = localizer.getCurrentPosition();
            telemetry.addData("x: ", pos.x);
            telemetry.addData("y: ", pos.y);
            telemetry.update();
        }

        fr_esq.setPower(0.0);
        fr_dir.setPower(0.0);
        ba_esq.setPower(0.0);
        ba_dir.setPower(0.0);

    }
}
