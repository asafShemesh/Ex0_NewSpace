
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

    public Double[] sideEngines;     
    public double mainEngine;     

    public static final double MAIN_ENGINE_FORCE = 430; 
    public static final double SIDE_ENGINE_FORCE = 25;   
    public static final double MAIN_ENGINE_BURN = 0.15;
    public static final double SIDE_ENGINE_BURN = 0.009; 

    // PID controller constants (shared by controllers)
    public double Kp = 0.05;
    public double Ki = 0.01;
    public double Kd = 0.2;

    // PID state variables for vertical speed control
    private double accumulatedErrorVertical = 0.0;
    private double previousErrorVertical = 0.0;

    // PID state variables for angle control (currently unused)
    private double accumulatedErrorAngle = 0.0;
    private double previousErrorAngle = 0.0;

    public Ship() {
        this.fuel = 121;
        baseMass = 165;
        this.mass = fuel + baseMass;

        this.angle = Math.toRadians(-80);
        this.horizontalSpeed = 932;
        this.verticalSpeed = 24.8;
        this.rotationSpeed = 0.0;
        this.altitude = 13748;

        this.time = 0;
        this.sideEngines = new Double[4];
        for (int i = 0; i < 4; i++) {
            this.sideEngines[i] = 0.0;
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
        this.sideEngines = new Double[4];
        for (int i = 0; i < 4; i++) {
            this.sideEngines[i] = 0.0;
        }
        this.mainEngine = 0.0;
    }


    public double calculatePIDVertical(double setPoint, double currentValue, double dt) {
        double error = setPoint - currentValue;
        accumulatedErrorVertical += error * dt;
        double derivative = (error - previousErrorVertical) / dt;
        previousErrorVertical = error;
        double P = Kp * error;
        double I = Ki * accumulatedErrorVertical;
        double D = Kd * derivative;
        double PID = P + I + D;
        System.out.printf("Vertical PID: P: %.4f, I: %.4f, D: %.4f, Output: %.4f\n", P, I, D, PID);
        return PID;
    }

    public double calculatePIDAangle(double setPoint, double currentValue, double dt) {
        double error = setPoint - currentValue;
        accumulatedErrorAngle += error * dt;
        double derivative = (error - previousErrorAngle) / dt;
        previousErrorAngle = error;
        double P = Kp * error;
        double I = Ki * accumulatedErrorAngle;
        double D = Kd * derivative;
        double PID = P + I + D;
        System.out.printf("Angle PID: P: %.4f, I: %.4f, D: %.4f, Output: %.4f\n", P, I, D, PID);
        return PID;
    }


    public void print() {
        System.out.println(time + ", verticalSpeed: " + verticalSpeed +
                           ", horizontalSpeed: " + horizontalSpeed +
                           ", altitude: " + altitude +
                           ", angle (deg): " + Math.toDegrees(angle) +
                           ", mass: " + mass);
    }



    public static double gravitationalAcceleration() {
        return 1.622;
    }


    public void update(double dt) {
        rotationAcceleration = 0.0;
        verticalAcceleration = 0.0;
        horizontalAcceleration = 0.0;
    
        double mainFuelUsed = dt * mainEngine * MAIN_ENGINE_BURN;
        boolean mainEngineActive = false;
        if (fuel >= mainFuelUsed && mainFuelUsed > 0) {
            fuel -= mainFuelUsed;
            mass = baseMass + fuel; 
            mainEngineActive = true;
        }
    
        if (mainEngineActive) {
            double mainThrust = mainEngine * MAIN_ENGINE_FORCE;
            double sideSetting = (sideEngines[0]+sideEngines[1]+sideEngines[2]+sideEngines[3]) * SIDE_ENGINE_FORCE;
            double sideThrust = sideSetting * SIDE_ENGINE_FORCE * 4;
            
            double totalThrust = mainThrust + sideThrust;
            double totalAcceleration = totalThrust / mass;
            
            verticalAcceleration += totalAcceleration * Math.cos(angle);
            horizontalAcceleration += totalAcceleration * Math.sin(angle);
        }
    
        verticalAcceleration -= gravitationalAcceleration();
        verticalSpeed += verticalAcceleration * dt;
        horizontalSpeed += horizontalAcceleration * dt;
        
    double newVerticalSpeed = verticalSpeed + verticalAcceleration * dt;
    altitude -= verticalSpeed * dt + 0.5 * verticalAcceleration * dt * dt;
    verticalSpeed = newVerticalSpeed;
    if (altitude < 0) {
        altitude = 0;

    }

        time += dt;
    }
}
    