using System;

namespace RocketScienceCleaned
{
    public class Optimizer
    {
        private readonly Rocket rocket;
        private readonly TrajectoryCalculator calculator;
        private readonly FlightTracker tracker;

        public Optimizer(Rocket rocket, TrajectoryCalculator calculator, FlightTracker tracker)
        {
            this.rocket = rocket;
            this.calculator = calculator;
            this.tracker = tracker;
        }

        public double OptimizeLandingFuel()
        {
            double currentLandingFuel = rocket.TotalFuel * 0.32;
            double learningRate = 0.01;
            double tolerance = 1e-6;
            double previousCost = double.MaxValue;
            double currentCost = 0;
            double previousLandingFuel = currentLandingFuel;
            const int maxIterations = 1000;

            tracker.StartOptimization();

            Console.WriteLine("Initializing optimization parameters:");
            Console.WriteLine("  Initial landing fuel guess: " + currentLandingFuel.ToString("F2") + " kg");
            Console.WriteLine("  Learning rate: " + learningRate.ToString("F6"));
            Console.WriteLine("  Convergence tolerance: " + tolerance.ToString("E"));
            Console.WriteLine("  Maximum iterations: " + maxIterations);
            Console.WriteLine();

            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                Console.WriteLine("ITERATION " + iteration.ToString("D4") + " START");
                Console.WriteLine("Current landing fuel parameter: " + currentLandingFuel.ToString("F6") + " kg");
                Console.WriteLine();

                Console.WriteLine("Evaluating cost function at current point...");
                currentCost = calculator.CalculateCost(currentLandingFuel, out FlightParameters currentParams, true);

                if (currentCost == double.MaxValue)
                {
                    tracker.LogInvalidSolution(iteration, currentLandingFuel);
                    if (iteration == 0)
                    {
                        currentLandingFuel = rocket.TotalFuel * 0.32;
                        previousLandingFuel = currentLandingFuel;
                        continue;
                    }
                    if (Math.Abs(currentLandingFuel - previousLandingFuel) < 1000)
                    {
                        currentLandingFuel = previousLandingFuel;
                        break;
                    }
                    currentLandingFuel = previousLandingFuel;
                    continue;
                }

                Console.WriteLine();
                Console.WriteLine("Computing analytical gradient...");
                
                double gradient = CalculateAnalyticalGradient(currentLandingFuel, currentParams, out double costPlus, out double costMinus);
                currentParams.Gradient = gradient;

                Console.WriteLine("Gradient calculation:");
                Console.WriteLine("  Forward perturbation cost: " + costPlus.ToString("F2"));
                Console.WriteLine("  Backward perturbation cost: " + costMinus.ToString("F2"));
                Console.WriteLine("  Cost difference: " + (costPlus - costMinus).ToString("F6"));
                Console.WriteLine("  Analytical gradient: " + gradient.ToString("F6"));
                Console.WriteLine();

                tracker.UpdateOptimizationProgress(iteration, currentLandingFuel, currentCost, previousCost, currentParams);

                previousLandingFuel = currentLandingFuel;
                double parameterUpdate = learningRate * gradient;
                currentLandingFuel -= parameterUpdate;
                double originalUpdate = currentLandingFuel;
                currentLandingFuel = Math.Max(0, Math.Min(currentLandingFuel, rocket.TotalFuel));

                Console.WriteLine("Parameter update:");
                Console.WriteLine("  Previous landing fuel: " + previousLandingFuel.ToString("F6") + " kg");
                Console.WriteLine("  Update amount: " + parameterUpdate.ToString("F6") + " kg (learning rate * gradient)");
                Console.WriteLine("  Unconstrained new value: " + originalUpdate.ToString("F6") + " kg");
                
                if (currentLandingFuel != originalUpdate)
                {
                    Console.WriteLine("  Constraint applied: Clamped to bounds [0, " + rocket.TotalFuel.ToString("F2") + "]");
                }
                
                Console.WriteLine("  New landing fuel: " + currentLandingFuel.ToString("F6") + " kg");
                Console.WriteLine("  Parameter change: " + (currentLandingFuel - previousLandingFuel).ToString("F6") + " kg");
                Console.WriteLine();

                if (iteration > 0)
                {
                    double costChange = currentCost - previousCost;
                    Console.WriteLine("Convergence check:");
                    Console.WriteLine("  Previous cost: " + previousCost.ToString("F6"));
                    Console.WriteLine("  Current cost: " + currentCost.ToString("F6"));
                    Console.WriteLine("  Cost change: " + costChange.ToString("F6"));
                    Console.WriteLine("  Absolute cost change: " + Math.Abs(costChange).ToString("E"));
                    Console.WriteLine("  Tolerance: " + tolerance.ToString("E"));
                    
                    if (Math.Abs(costChange) < tolerance)
                    {
                        Console.WriteLine("  Convergence criterion met!");
                        Console.WriteLine();
                        tracker.LogConvergence(iteration);
                        break;
                    }
                    else
                    {
                        Console.WriteLine("  Continuing optimization...");
                    }
                }
                else
                {
                    Console.WriteLine("First iteration: No convergence check");
                }

                previousCost = currentCost;
                Console.WriteLine();
                Console.WriteLine("ITERATION " + iteration.ToString("D4") + " COMPLETE");
                Console.WriteLine();
            }

            tracker.CompleteOptimization();
            return currentLandingFuel;
        }

        private double CalculateAnalyticalGradient(double landingFuel, FlightParameters currentParams, out double costPlus, out double costMinus)
        {
            double currentCost = currentParams.Cost;
            double delta = Math.Max(1000.0, landingFuel * 0.002);
            costPlus = calculator.CalculateCost(landingFuel + delta, out _, false);
            costMinus = calculator.CalculateCost(landingFuel - delta, out _, false);
            
            if (costPlus == double.MaxValue && costMinus == double.MaxValue)
            {
                return 0;
            }
            
            if (costPlus == double.MaxValue)
            {
                if (costMinus != double.MaxValue && costMinus < currentCost)
                {
                    return (costMinus - currentCost) / delta;
                }
                return 0.01;
            }
            
            if (costMinus == double.MaxValue)
            {
                if (costPlus != double.MaxValue && costPlus < currentCost)
                {
                    return (currentCost - costPlus) / delta;
                }
                return -0.01;
            }
            
            double gradient = (costPlus - costMinus) / (2 * delta);
            
            if (Math.Abs(gradient) < 1e-10)
            {
                if (costPlus < currentCost)
                {
                    return -0.001;
                }
                if (costMinus < currentCost)
                {
                    return 0.001;
                }
            }
            
            return gradient;
        }
    }
}
