public class algo {

    public static boolean flag = false;
    private static final double DT = 1.0;

    public static void main(String[] args) {
        Ship bereshit = new Ship();

        bereshit.Kp = 0.05;
        bereshit.Ki = 0.002;
        bereshit.Kd = 0.2;

        while (bereshit.altitude > 0) {
            controlLogic(bereshit, DT);

            bereshit.update(DT);

            bereshit.print();
        }

        System.out.println("Landed!");
    }

    public static void controlLogic(Ship bereshit, double dt) {
        // Stage 1: Handle horizontal speed first
        if (Math.abs(bereshit.horizontalSpeed) > 10) {
            // Tilt the ship to push against horizontal motion
            double targetAngle = 0;
                targetAngle = Math.toRadians(-50.379); // Push left
        if (bereshit.verticalSpeed>-0.6) {
            targetAngle = Math.toRadians(-58); // Push left            
        }



            // Use PID to calculate how much thrust is needed
            double desiredHSpeed = 0.0;
            double currentHSpeed = bereshit.horizontalSpeed;
            double pidHorizontal = bereshit.calculatePIDHorizontal(desiredHSpeed, currentHSpeed, dt, false);

            // Clamp the throttle between 0 and 1
            double horizontalThrottle = Math.max(0.0, Math.min(1.0, Math.abs(pidHorizontal)));
            //horizontalThrottle = 1.0;

            // Apply PID throttle to main engine and side engines
            bereshit.mainEngine = horizontalThrottle;

            // Use side engines to assist (optional: adjust which ones based on thrust vectoring)

            if (Math.abs(targetAngle - bereshit.angle) < Math.toRadians(4)) {
                bereshit.angle = targetAngle;
                for (int i = 0; i < bereshit.rightSideEngines.length; i++) {
                    bereshit.rightSideEngines[i] = horizontalThrottle;
                    bereshit.leftSideEngines[i] = horizontalThrottle;
                }
            }
            else if (targetAngle - bereshit.angle > Math.toRadians(4)) {
                for (int i = 0; i < bereshit.rightSideEngines.length; i++) {
                    bereshit.rightSideEngines[i] = 1.0;
                }
                for (int i = 0; i < bereshit.leftSideEngines.length; i++) {
                    bereshit.leftSideEngines[i] = 0.0;
                }
            }
            else if (bereshit.angle - targetAngle > Math.toRadians(4)) {
                for (int i = 0; i < bereshit.rightSideEngines.length; i++) {
                    bereshit.rightSideEngines[i] = 0.0;
                }
                for (int i = 0; i < bereshit.leftSideEngines.length; i++) {
                    bereshit.leftSideEngines[i] = 1.0;
                }
            }

        } else {
            // Stage 2: Vertical landing
            double targetAngle = Math.toRadians(0); // Straighten up

            // Use PID to manage vertical speed
            double desiredVSpeed = getDesiredVerticalSpeed(bereshit.altitude);
            double currentVSpeed = bereshit.verticalSpeed;
            if (!flag && bereshit.verticalSpeed < -20.0){
                bereshit.resetPID();
                flag=true;
            }
            double pidVertical = bereshit.calculatePIDVertical(desiredVSpeed, currentVSpeed, dt, false);

            // Clamp throttle between 0 and 1
            double throttle = Math.max(0.0, Math.min(1.0, pidVertical));
            bereshit.mainEngine = throttle;

            if (Math.abs(targetAngle - bereshit.angle) < Math.toRadians(4)) {
                bereshit.angle = targetAngle;
                for (int i = 0; i < bereshit.rightSideEngines.length; i++) {
                    bereshit.rightSideEngines[i] = throttle;
                    bereshit.leftSideEngines[i] = throttle;
                }
            }
            else if (targetAngle - bereshit.angle > Math.toRadians(4)) {
                for (int i = 0; i < bereshit.rightSideEngines.length; i++) {
                    bereshit.rightSideEngines[i] = 1.0;
                }
                for (int i = 0; i < bereshit.leftSideEngines.length; i++) {
                    bereshit.leftSideEngines[i] = 0.0;
                }
            }
            else if (bereshit.angle - targetAngle > Math.toRadians(4)) {
                for (int i = 0; i < bereshit.rightSideEngines.length; i++) {
                    bereshit.rightSideEngines[i] = 0.0;
                }
                for (int i = 0; i < bereshit.leftSideEngines.length; i++) {
                    bereshit.leftSideEngines[i] = 1.0;
                }
            }
        }
    }

    private static double getDesiredVerticalSpeed(double altitude) {
        if (altitude > 300) {
            return -40.0;
        } else if (altitude > 100) {
            return -5;
        } else if (altitude > 30) {
            return -1;

        } else {
            return 0;
        }
    }
}
