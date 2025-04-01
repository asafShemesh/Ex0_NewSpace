public class algo {

    private static final double DT = 1.0;

    public static void main(String[] args) {
        Ship bereshit = new Ship();

        bereshit.Kp = 0.05;
        bereshit.Ki = 0.01;
        bereshit.Kd = 0.2;

        while (bereshit.altitude > 0) {
            controlLogic(bereshit, DT);

            bereshit.update(DT);

            bereshit.print();
        }

        System.out.println("Landed!");
    }

    
    private static double getSideEngineThrust(Ship bereshit,double absHorizontalSpeed) {
        if (absHorizontalSpeed >20) {
            return 0.5;
        } else {
            bereshit.angle= Math.toRadians(-11);
            return 0.017;
    }
}


    public static void controlLogic(Ship bereshit, double dt) {
        double desiredVSpeed = getDesiredVerticalSpeed(bereshit.altitude);
        double currentVSpeed = bereshit.verticalSpeed;
        double pidVertical = bereshit.calculatePIDVertical(desiredVSpeed, currentVSpeed, dt);
        double throttle = Math.max(0.0, Math.min(1.0, pidVertical));
        bereshit.mainEngine = throttle;

        double sideThrust = getSideEngineThrust(bereshit,bereshit.horizontalSpeed);
        bereshit.mainEngine=sideThrust;
        for (int i = 0; i < bereshit.sideEngines.length; i++) {
            bereshit.sideEngines[i] = sideThrust;
        }
    }


    private static double getDesiredVerticalSpeed(double altitude) {
        if (altitude > 2000) {
            return -20.0;
        } else if (altitude > 500) {
            return -10.0;
        } else {
            return -3.0;
        }
    }
}
