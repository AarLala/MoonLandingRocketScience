using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

namespace RocketScienceCleaned
{
    public class OptimizationValidator
    {
        private readonly Rocket rocket;
        private readonly TrajectoryCalculator calculator;

        public OptimizationValidator(Rocket rocket, TrajectoryCalculator calculator)
        {
            this.rocket = rocket;
            this.calculator = calculator;
        }

        public (double J, Dictionary<string, double> metrics) EvaluateLandingFuel(double mLanding)
        {
            double J = calculator.CalculateCost(mLanding, out FlightParameters parameters, false);
            
            Dictionary<string, double> metrics = new Dictionary<string, double>();
            
            if (J == double.MaxValue)
            {
                return (double.MaxValue, metrics);
            }
            
            double totalFuelUsed = parameters.LaunchFuel + mLanding;
            double landingFuelUsed = mLanding;
            double finalMass = rocket.Payload;
            
            metrics["fuel_used"] = totalFuelUsed;
            metrics["v_final"] = parameters.FinalVelocity;
            metrics["m_final"] = finalMass;
            metrics["t_final"] = parameters.TotalFlightTime;
            metrics["speed_penalty"] = parameters.CostPenalty;
            
            return (J, metrics);
        }

        public void PerformSensitivitySweep(double m0, string outputPath)
        {
            int nPoints = 60;
            double[] mValues = new double[nPoints];
            double[] JValues = new double[nPoints];
            double[] fuelUsedValues = new double[nPoints];
            double[] vFinalValues = new double[nPoints];
            
            for (int i = 0; i < nPoints; i++)
            {
                double m = 0.8 * m0 + (1.2 * m0 - 0.8 * m0) * i / (nPoints - 1);
                mValues[i] = m;
                
                var (J, metrics) = EvaluateLandingFuel(m);
                JValues[i] = J == double.MaxValue ? double.NaN : J;
                
                if (metrics.Count > 0)
                {
                    fuelUsedValues[i] = metrics["fuel_used"];
                    vFinalValues[i] = metrics["v_final"];
                }
                else
                {
                    fuelUsedValues[i] = double.NaN;
                    vFinalValues[i] = double.NaN;
                }
            }
            
            string csvPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Desktop), "sensitivity_sweep.csv");
            using (StreamWriter writer = new StreamWriter(csvPath))
            {
                writer.WriteLine("m_landing,J,fuel_used,v_final");
                for (int i = 0; i < nPoints; i++)
                {
                    writer.WriteLine($"{mValues[i]:F6},{JValues[i]:F6},{fuelUsedValues[i]:F6},{vFinalValues[i]:F6}");
                }
            }
            
            Console.WriteLine($"Sensitivity sweep data saved to: {csvPath}");
        }

        public double ComputeFiniteDifferenceGradient(double m0)
        {
            double eps = 1e-3 * m0;
            
            var (JPlus, _) = EvaluateLandingFuel(m0 + eps);
            var (JMinus, _) = EvaluateLandingFuel(m0 - eps);
            
            if (JPlus == double.MaxValue || JMinus == double.MaxValue)
            {
                return double.NaN;
            }
            
            double gFd = (JPlus - JMinus) / (2 * eps);
            return gFd;
        }

        public List<int> FindParetoFront(double[] fuelUsed, double[] vFinal)
        {
            int n = fuelUsed.Length;
            List<int> paretoIndices = new List<int>();
            
            for (int i = 0; i < n; i++)
            {
                if (double.IsNaN(fuelUsed[i]) || double.IsNaN(vFinal[i]))
                    continue;
                
                bool isPareto = true;
                
                for (int j = 0; j < n; j++)
                {
                    if (i == j || double.IsNaN(fuelUsed[j]) || double.IsNaN(vFinal[j]))
                        continue;
                    
                    if (fuelUsed[j] <= fuelUsed[i] && vFinal[j] <= vFinal[i])
                    {
                        if (fuelUsed[j] < fuelUsed[i] || vFinal[j] < vFinal[i])
                        {
                            isPareto = false;
                            break;
                        }
                    }
                }
                
                if (isPareto)
                {
                    paretoIndices.Add(i);
                }
            }
            
            return paretoIndices;
        }

        public void SaveOptimizerHistory(List<double> mHistory, List<double> JHistory, string outputPath)
        {
            string csvPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Desktop), "optimizer_history.csv");
            using (StreamWriter writer = new StreamWriter(csvPath))
            {
                writer.WriteLine("iteration,m_landing,J");
                for (int i = 0; i < mHistory.Count; i++)
                {
                    writer.WriteLine($"{i},{mHistory[i]:F6},{JHistory[i]:F6}");
                }
            }
            
            Console.WriteLine($"Optimizer history saved to: {csvPath}");
        }
    }
}

