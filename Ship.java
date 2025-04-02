
public class Ship {
    public double fuel;
    private double baseMass;
    public double mass;

    public double angle;      
    public double horizontalSpeed;
    public double verticalSpeed;
    public double rotationSpeed;

    public double horizontalAcceleration;
    public double verticalAcceleration;
    public double rotationAcceleration;

    public double altitude;
    public double time;

    public Double[] rightSideEngines;
    public Double[] leftSideEngines;
    public double mainEngine;     

    public static final double MAIN_ENGINE_FORCE = 430; 
    public static final double SIDE_ENGINE_FORCE = 25;   
    public static final double MAIN_ENGINE_BURN = 0.15;
    public static final double SIDE_ENGINE_BURN = 0.009; 

    // PID controller constants (shared by controllers)
    public double Kp = 0.05;
    public double Ki = 0.00002;
    public double Kd = 0.2;

    // PID state variables for vertical speed control
    private double accumulatedErrorVertical = 0.0;
    private double previousErrorVertical = 0.0;

    // PID state variables for horizontal speed control
    private double accumulatedErrorHorizontal = 0.0;
    private double previousErrorHorizontal = 0.0;

    public Ship() {
        this.fuel = 121;
        baseMass = 165;
        this.mass = fuel + baseMass;

        this.angle = Math.toRadians(-60);
        this.horizontalSpeed = 932;
        this.verticalSpeed = -24.8;
        this.rotationSpeed = 0.0;
        this.altitude = 13748;

        this.time = 0;
        this.rightSideEngines = new Double[4];
        this.leftSideEngines = new Double[4];
        for (int i = 0; i < 4; i++) {
            this.rightSideEngines[i] = 0.0;
            this.leftSideEngines[i] = 0.0;
        }
        this.mainEngine = 0.0;
    }

    public Ship(double fuel, double baseMass, double angle,
                double horizontalSpeed, double verticalSpeed, double rotationSpeed,
                double altitude) {
        this.fuel = fuel;
        this.baseMass = baseMass;
        this.mass = baseMass + fuel;

        this.angle = angle;
        this.horizontalSpeed = horizontalSpeed;
        this.verticalSpeed = verticalSpeed;
        this.rotationSpeed = rotationSpeed;
        this.altitude = altitude;

        this.time = 0;
        this.rightSideEngines = new Double[4];
        this.leftSideEngines = new Double[4];
        for (int i = 0; i < 4; i++) {
            this.rightSideEngines[i] = 0.0;
            this.leftSideEngines[i] = 0.0;
        }
        this.mainEngine = 0.0;
    }


    public double calculatePIDVertical(double setPoint, double currentValue, double dt, boolean print) {
        double error = setPoint - currentValue;
        accumulatedErrorVertical += error * dt;
        double derivative = (error - previousErrorVertical) / dt;
        previousErrorVertical = error;
    
        double P = Kp * error;
        double I = Ki * accumulatedErrorVertical;
        double D = Kd * derivative;
    
        double PID = P + I + D;
    
        if (print)
            System.out.printf("Vertical PID: P: %.4f, I: %.4f, D: %.4f, Output: %.4f\n", P, I, D, PID);
    
        return PID;
    }
    

    public double calculatePIDHorizontal(double setPoint, double currentValue, double dt, Boolean print) {
        double error = setPoint - currentValue;
        accumulatedErrorHorizontal += error * dt;
        double derivative = (error - previousErrorHorizontal) / dt;
        previousErrorHorizontal = error;

        double P = Kp * error;
        double I = Ki * accumulatedErrorHorizontal;
        double D = Kd * derivative;

        double PID = P + I + D;

        if (print)
            System.out.printf("Horizontal PID: P: %.4f, I: %.4f, D: %.4f, Output: %.4f\n", P, I, D, PID);

        return PID;
    }

    public void resetPID(){
        accumulatedErrorVertical=0.0;
        previousErrorVertical=0.0;
    }

    public void print() {
        System.out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                time, verticalSpeed, horizontalSpeed, altitude, Math.toDegrees(angle), fuel, mainEngine);
    }

    public static double gravitationalAcceleration() {
        return 1.622;
    }

    public void update(double dt) {
        rotationAcceleration = 0.0;
        verticalAcceleration = 0.0;
        horizontalAcceleration = 0.0;
        
        double sideSetting = 0;
            for (double side : rightSideEngines) {
                sideSetting += side;
            }
            for (double side : leftSideEngines) {
                sideSetting += side;
            }

        double fuelUsed = (dt * mainEngine * MAIN_ENGINE_BURN) +(SIDE_ENGINE_BURN * sideSetting *dt);
        boolean engineActive = false;
        if (fuel >= fuelUsed) {
            fuel -= fuelUsed;
            mass = baseMass + fuel;
            engineActive = true;
        }

        if (engineActive) {
            

            double totalThrust = mainEngine * MAIN_ENGINE_FORCE + sideSetting * SIDE_ENGINE_FORCE;
            double totalAcceleration = totalThrust / mass;
            
            verticalAcceleration += totalAcceleration * Math.cos(angle);
            horizontalAcceleration += totalAcceleration * Math.sin(angle);

            for (double side : rightSideEngines) {
                rotationAcceleration -= side;
            }
            for (double side : leftSideEngines) {
                rotationAcceleration += side;
            }
            rotationAcceleration *= Math.toRadians(-1);
        }
    
        verticalAcceleration -= gravitationalAcceleration();
        verticalSpeed += verticalAcceleration * dt;
        horizontalSpeed += horizontalAcceleration * dt;
        angle += rotationAcceleration * dt;

        altitude += verticalSpeed * dt;

        if (altitude < 0) {
            altitude = 0;
        }

        time += dt;
    }
}