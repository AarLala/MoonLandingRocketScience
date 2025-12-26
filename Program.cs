using System;
using System.Diagnostics;

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
                landingSpeedTarget: 3.0,
                dragCoefficient: 0.3,
                crossSectionalArea: 50.0
            );

            FlightTracker tracker = new FlightTracker(rocket);
            TrajectoryCalculator calculator = new TrajectoryCalculator(rocket, tracker);
            Optimizer optimizer = new Optimizer(rocket, calculator, tracker);
            OptimizationValidator validator = new OptimizationValidator(rocket, calculator);

            double initialGuess = rocket.TotalFuel * 0.32;

            Console.WriteLine("Starting optimization...");
            Console.WriteLine($"Initial fuel allocation: {initialGuess:F0} kg ({initialGuess / rocket.TotalFuel * 100:F1}% of total fuel)");
            Console.WriteLine();

            double optimalLandingFuel = optimizer.OptimizeLandingFuel(initialGuess);

            calculator.CalculateCost(optimalLandingFuel, out FlightParameters finalParameters, false);

            tracker.DisplayFinalResults(optimalLandingFuel, finalParameters);

            Console.WriteLine();
            Console.WriteLine("Validation Analysis");
            Console.WriteLine();

            Console.WriteLine("Computing gradient at optimal point...");
            double gFd = validator.ComputeFiniteDifferenceGradient(optimalLandingFuel);
            Console.WriteLine($"Optimal landing fuel: {optimalLandingFuel:F0} kg");
            Console.WriteLine($"Gradient: {gFd:E6}");
            Console.WriteLine();

            Console.WriteLine("Performing sensitivity analysis...");
            validator.PerformSensitivitySweep(optimalLandingFuel, "");
            Console.WriteLine();

            Console.WriteLine("Saving optimization history...");
            validator.SaveOptimizerHistory(optimizer.GetMHistory(), optimizer.GetJHistory(), "");
            Console.WriteLine();

            Console.WriteLine("All results saved to desktop.");
        }

    }
}
