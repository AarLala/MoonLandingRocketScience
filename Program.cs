namespace RocketScienceCleaned
{
    class Program
    {
        static void Main()
        {
            Rocket rocket = new Rocket(
                thrust: 4.5e7,
                burnRate: 1800.0,
                totalFuel: 2600000.0,
                payload: 50000.0,
                landingSpeedTarget: 1.0,
                dragCoefficient: 0.3,
                crossSectionalArea: 50.0
            );

            FlightTracker tracker = new FlightTracker(rocket);
            TrajectoryCalculator calculator = new TrajectoryCalculator(rocket, tracker);
            Optimizer optimizer = new Optimizer(rocket, calculator, tracker);

            double optimalLandingFuel = optimizer.OptimizeLandingFuel();

            calculator.CalculateCost(optimalLandingFuel, out FlightParameters finalParameters, false);

            tracker.DisplayFinalResults(optimalLandingFuel, finalParameters);
        }
    }
}
