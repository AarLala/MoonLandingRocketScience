using System;
using System.Collections.Generic;
using System.IO;

namespace RocketScienceCleaned
{
    public class Optimizer
    {
        private readonly Rocket rocket;
        private readonly TrajectoryCalculator calculator;
        private readonly FlightTracker tracker;
        private readonly string csvFilePath;
        private List<double> mHistory;
        private List<double> JHistory;

        public Optimizer(Rocket rocket, TrajectoryCalculator calculator, FlightTracker tracker)
        {
            this.rocket = rocket;
            this.calculator = calculator;
            this.tracker = tracker;
            this.mHistory = new List<double>();
            this.JHistory = new List<double>();
            
            string desktopPath = Environment.GetFolderPath(Environment.SpecialFolder.Desktop);
            this.csvFilePath = Path.Combine(desktopPath, "optimization_data.csv");
            File.WriteAllText(csvFilePath, "iteration,landing_fuel,cost\n");
            Console.WriteLine($"Saving optimization data to {csvFilePath}");
        }

        public double OptimizeLandingFuel(double initialGuess = double.NaN)
        {
            const double xmin = 0.30;
            const double xmax = 0.60;
            const double defaultRatio = 0.32;
            const double gradTol = 1e-2;
            const double speedTol = 3.5;
            const int maxIterations = 1000;
            
            double xInit = double.IsNaN(initialGuess) ? defaultRatio : (initialGuess / rocket.TotalFuel);
            double y = Math.Log((xInit - xmin) / (xmax - xInit));
            double x = Map(y, xmin, xmax);
            double J = calculator.CalculateCost(x * rocket.TotalFuel, out _, false);
            double alpha = 0.05;
            
            tracker.StartOptimization();
            
            for (int iter = 0; iter < maxIterations; iter++)
            {
                double g = ComputeGradient(y, J, xmin, xmax);

                if (!double.IsFinite(J) || !double.IsFinite(g))
                {
                    alpha *= 0.5;
                    continue;
                }

                calculator.CalculateCost(x * rocket.TotalFuel, out FlightParameters checkParams, false);
                double actualLandingSpeed = checkParams.FinalVelocity;
                
                if (Math.Abs(g) < gradTol && actualLandingSpeed <= speedTol)
                {
                    Console.WriteLine($"Optimization converged after {iter} iterations");
                    Console.WriteLine($"Landing speed: {actualLandingSpeed:F4} m/s (target: ≤{speedTol:F1} m/s)");
                    break;
                }
                
                if (Math.Abs(g) < gradTol && actualLandingSpeed > speedTol)
                {
                    Console.WriteLine($"Gradient converged but landing speed too high ({actualLandingSpeed:F4} m/s > {speedTol:F1} m/s), continuing optimization...");
                }

                double gNorm = g / (Math.Abs(g) + 1.0);
                bool accepted = false;
                double alphaTry = alpha;

                for (int ls = 0; ls < 20; ls++)
                {
                    double yNew = y - alphaTry * gNorm;
                    double xNew = Map(yNew, xmin, xmax);
                    double JNew = calculator.CalculateCost(xNew * rocket.TotalFuel, out _, false);

                    if (double.IsFinite(JNew) && JNew <= J)
                    {
                        y = yNew;
                        x = xNew;
                        J = JNew;
                        accepted = true;
                        break;
                    }
                    alphaTry *= 0.5;
                }

                if (!accepted)
                {
                    alpha *= 0.5;
                    if (alpha < 1e-12)
                    {
                        Console.WriteLine($"Step size became too small, stopping optimization");
                        break;
                    }
                }
                
                LogToCSV(iter, x * rocket.TotalFuel, J);
                mHistory.Add(x * rocket.TotalFuel);
                JHistory.Add(J);
            }

            tracker.CompleteOptimization();
            double finalX = Map(y, xmin, xmax);
            return finalX * rocket.TotalFuel;
        }

        private static double Map(double y, double xmin, double xmax)
        {
            return xmin + (xmax - xmin) / (1 + Math.Exp(-y));
        }

        public List<double> GetMHistory() => mHistory;
        public List<double> GetJHistory() => JHistory;

        private double ComputeGradient(double y, double currentCost, double xmin, double xmax)
        {
            double eps = 1e-4;
            
            for (int retry = 0; retry < 5; retry++)
            {
                double yPlus = y + eps;
                double yMinus = y - eps;
                
                double xPlus = Map(yPlus, xmin, xmax);
                double xMinus = Map(yMinus, xmin, xmax);
                
                double landingFuelPlus = xPlus * rocket.TotalFuel;
                double landingFuelMinus = xMinus * rocket.TotalFuel;
                
                double costPlus = calculator.CalculateCost(landingFuelPlus, out _, false);
                double costMinus = calculator.CalculateCost(landingFuelMinus, out _, false);
                
                if (double.IsFinite(costPlus) && double.IsFinite(costMinus))
                {
                    double gradient = (costPlus - costMinus) / (2 * eps);
                    if (double.IsFinite(gradient))
                    {
                        return gradient;
                    }
                }
                
                eps *= 0.1;
            }
            
            return 0.0;
        }

        private void LogToCSV(int iteration, double landingFuel, double cost)
        {
            string line = $"{iteration},{landingFuel:F6},{cost:F6}\n";
            File.AppendAllText(csvFilePath, line);
        }
    }
}
