package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auxiliar.DriveConstants;
import org.firstinspires.ftc.teamcode.Auxiliar.Vector2D;

public class Localizer
{
    Encoder parallelEncoder;
    Encoder lateralEncoder;

    Vector2D initialPosition = new Vector2D(0,0);
    double initialAngle = 0;

    IMU imu;

    // Paralela, Lateral
    int[] encoderTicks;
    int[] lastEncoderTicks = new int[] {0,0};

    double lastAngle = 0;
    public double currentAngle;

    Vector2D currentPosition = initialPosition;

    public Localizer(HardwareMap hardwareMap)
    {
        // MUDAR AS PORTAS DOS ENCODERES
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class,
                "left_front_drive"));
        lateralEncoder = new Encoder(hardwareMap.get(DcMotorEx.class,
                "right_back_drive"));

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(DriveConstants.imuParameters);
        imu.resetYaw();
    }

    public void update()
    {

        encoderTicks = getEncoderTicks();

        double parallelDistanceToCenter = 25;
        double lateralDistanceToCenter = 21;

        double parallelDisplacement = (encoderTicks[0] - lastEncoderTicks[0]) * Encoder.TICKS2CM;
        double lateralDisplacement = (encoderTicks[1] - lastEncoderTicks[1]) * Encoder.TICKS2CM;

        currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double angleDisplacement = currentAngle - lastAngle;

       // parallelDisplacement -= -parallelDistanceToCenter * angleDisplacement;
        //lateralDisplacement -= -lateralDistanceToCenter * angleDisplacement;

        Vector2D robotDisplacement = new Vector2D(0,0);

        if (angleDisplacement < 0.001)
        {
            robotDisplacement.x = (1 - Math.pow(angleDisplacement, 2) / 6) * parallelDisplacement - (angleDisplacement / 2) * lateralDisplacement;
            robotDisplacement.y = (angleDisplacement / 2) * parallelDisplacement + (1 - Math.pow(angleDisplacement, 2) / 6) * lateralDisplacement;
        } else {
            robotDisplacement.x = (Math.sin(angleDisplacement) / angleDisplacement) * parallelDisplacement + ((Math.cos(angleDisplacement) - 1) / angleDisplacement) * lateralDisplacement;
            robotDisplacement.y = ((1 - Math.cos(angleDisplacement)) / angleDisplacement) * parallelDisplacement + (Math.sin(angleDisplacement) / angleDisplacement) * lateralDisplacement;
        }

        currentPosition.x += lateralDisplacement;//robotDisplacement.x;
        currentPosition.y += parallelDisplacement; //robotDisplacement.y;

        lastEncoderTicks = encoderTicks;
        lastAngle = currentAngle;

    }

    public Vector2D getCurrentPosition()
    {
        return currentPosition;
    }

    int[] getEncoderTicks()
    {
        return new int[] {parallelEncoder.getPosition(), lateralEncoder.getPosition()};
    }


    

}
