using System;

namespace RocketScienceCleaned
{
    public class TrajectoryCalculator
    {
        private readonly Rocket rocket;
        private readonly FlightTracker tracker;

        public TrajectoryCalculator(Rocket rocket, FlightTracker tracker)
        {
            this.rocket = rocket;
            this.tracker = tracker;
        }

        public double CalculateCost(double landingFuel, out FlightParameters parameters, bool verbose = false)
        {
            if (verbose)
            {
                Console.WriteLine("  Computing cost function for landing fuel: " + landingFuel.ToString("F2") + " kg");
            }

            parameters = new FlightParameters();
            parameters.LandingFuel = landingFuel;
            parameters.LaunchFuel = rocket.TotalFuel - landingFuel;

            if (verbose)
            {
                Console.WriteLine("  Validating fuel allocation constraints...");
            }

            if (landingFuel <= 0 || landingFuel >= rocket.TotalFuel)
            {
                if (verbose)
                {
                    Console.WriteLine("  Validation failed: Landing fuel out of bounds");
                }
                parameters.Cost = double.MaxValue;
                return double.MaxValue;
            }

            if (verbose)
            {
                Console.WriteLine("  Validation passed: Fuel allocation within bounds");
                Console.WriteLine("  Calculating launch phase with atmospheric drag...");
            }

            double launchFuel = rocket.TotalFuel - landingFuel;
            LaunchPhaseResult launchResult = CalculateLaunchPhase(launchFuel, verbose);
            
            if (!launchResult.Success)
            {
                if (verbose)
                {
                    Console.WriteLine("  Launch phase failed: Insufficient delta-V");
                }
                parameters.Cost = double.MaxValue;
                return double.MaxValue;
            }

            parameters.LaunchTime = launchResult.Time;
            parameters.LaunchStopAltitude = launchResult.Altitude;
            parameters.LaunchPhaseDeltaV = launchResult.DeltaV;
            parameters.LaunchOrbitalAltitude = launchResult.Altitude;
            double achievedVelocity = launchResult.Velocity;

            if (verbose)
            {
                Console.WriteLine("  Launch phase complete. Orbital altitude: " + parameters.LaunchStopAltitude.ToString("F0") + " m");
                Console.WriteLine("  Launch delta-V achieved: " + parameters.LaunchPhaseDeltaV.ToString("F2") + " m/s");
            }

            double circularOrbitVelocityAtAltitude = CalculateCircularOrbitVelocity(PhysicalConstants.EarthRadius + parameters.LaunchOrbitalAltitude, PhysicalConstants.EarthMass);
            parameters.CircularOrbitVelocity = circularOrbitVelocityAtAltitude;

            if (verbose)
            {
                Console.WriteLine("  Circular orbit velocity at " + parameters.LaunchOrbitalAltitude.ToString("F0") + " m: " + circularOrbitVelocityAtAltitude.ToString("F2") + " m/s");
                Console.WriteLine("  Achieved velocity: " + achievedVelocity.ToString("F2") + " m/s");
            }

            double velocityRatio = achievedVelocity / circularOrbitVelocityAtAltitude;
            double minRequiredAltitude = PhysicalConstants.LowEarthOrbitAltitude * 0.8;
            if (velocityRatio < 0.88 || parameters.LaunchOrbitalAltitude < minRequiredAltitude)
            {
                if (verbose)
                {
                    Console.WriteLine("  Validation failed: Insufficient velocity for circular orbit");
                    Console.WriteLine("  Required velocity ratio: 0.95, Achieved: " + velocityRatio.ToString("F3"));
                    Console.WriteLine("  Required altitude: " + minRequiredAltitude.ToString("F0") + " m, Achieved: " + parameters.LaunchOrbitalAltitude.ToString("F0") + " m");
                }
                parameters.Cost = double.MaxValue;
                return double.MaxValue;
            }

            if (verbose)
            {
                Console.WriteLine("  Calculating Hohmann transfer orbit...");
            }

            TransferOrbitResult transferResult = CalculateHohmannTransfer(parameters.LaunchOrbitalAltitude, verbose);
            
            if (!transferResult.Success)
            {
                if (verbose)
                {
                    Console.WriteLine("  Transfer orbit calculation failed");
                }
                parameters.Cost = double.MaxValue;
                return double.MaxValue;
            }

            parameters.TransferOrbitPerigeeVelocity = transferResult.PerigeeVelocity;
            parameters.TransferOrbitApogeeVelocity = transferResult.ApogeeVelocity;
            parameters.TransferOrbitSemiMajorAxis = transferResult.SemiMajorAxis;
            parameters.TransferOrbitPeriod = transferResult.Period;
            parameters.CoastTime = transferResult.Period / 2.0;

            double transferDeltaV = transferResult.PerigeeVelocity - circularOrbitVelocityAtAltitude;
            double totalDeltaVSoFar = parameters.LaunchPhaseDeltaV + transferDeltaV;

            if (verbose)
            {
                Console.WriteLine("  Transfer orbit perigee velocity: " + transferResult.PerigeeVelocity.ToString("F2") + " m/s");
                Console.WriteLine("  Transfer orbit apogee velocity: " + transferResult.ApogeeVelocity.ToString("F2") + " m/s");
                Console.WriteLine("  Transfer orbit period: " + transferResult.Period.ToString("F2") + " seconds");
                Console.WriteLine("  Transfer delta-V required: " + transferDeltaV.ToString("F2") + " m/s");
            }

            if (verbose)
            {
                Console.WriteLine("  Calculating lunar orbit insertion...");
            }

            double lunarOrbitVelocity = CalculateCircularOrbitVelocity(PhysicalConstants.MoonRadius + PhysicalConstants.LunarOrbitAltitude, PhysicalConstants.MoonMass);
            parameters.LunarOrbitVelocity = lunarOrbitVelocity;
            parameters.LunarOrbitalAltitude = PhysicalConstants.LunarOrbitAltitude;

            double lunarOrbitInsertionDeltaV = Math.Abs(transferResult.ApogeeVelocity - lunarOrbitVelocity);
            parameters.LunarOrbitInsertionDeltaV = lunarOrbitInsertionDeltaV;

            if (verbose)
            {
                Console.WriteLine("  Lunar orbit velocity: " + lunarOrbitVelocity.ToString("F2") + " m/s");
                Console.WriteLine("  Lunar orbit insertion delta-V: " + lunarOrbitInsertionDeltaV.ToString("F2") + " m/s");
            }

            if (verbose)
            {
                Console.WriteLine("  Calculating landing descent from lunar orbit...");
            }

            LandingPhaseResult landingResult = CalculateLunarLanding(landingFuel, lunarOrbitVelocity, verbose);
            
            if (!landingResult.Success)
            {
                if (verbose)
                {
                    Console.WriteLine("  Landing phase failed: Insufficient fuel or thrust");
                }
                parameters.Cost = double.MaxValue;
                return double.MaxValue;
            }

            parameters.LandingTime = landingResult.Time;
            parameters.FinalVelocity = landingResult.FinalVelocity;
            parameters.LandingDescentDeltaV = landingResult.DeltaV;

            if (verbose)
            {
                Console.WriteLine("  Landing descent delta-V: " + landingResult.DeltaV.ToString("F2") + " m/s");
                Console.WriteLine("  Final landing velocity: " + landingResult.FinalVelocity.ToString("F4") + " m/s");
            }

            parameters.TotalDeltaV = parameters.LaunchPhaseDeltaV + transferDeltaV + lunarOrbitInsertionDeltaV + landingResult.DeltaV;

            double speedDeviation = Math.Max(0, Math.Abs(parameters.FinalVelocity) - rocket.LandingSpeedTarget);
            double penalty = Math.Pow(speedDeviation, 2) * 1e6;
            parameters.CostPenalty = penalty;

            if (verbose)
            {
                Console.WriteLine("  Landing speed deviation: " + speedDeviation.ToString("F4") + " m/s");
                Console.WriteLine("  Speed penalty: " + penalty.ToString("F2"));
                Console.WriteLine("  Total mission delta-V: " + parameters.TotalDeltaV.ToString("F2") + " m/s");
            }

            double totalCost = parameters.LaunchTime + parameters.CoastTime + parameters.LandingTime + penalty;
            parameters.Cost = totalCost;

            if (verbose)
            {
                Console.WriteLine("  Cost components: Launch=" + parameters.LaunchTime.ToString("F2") + 
                                "s, Coast=" + parameters.CoastTime.ToString("F2") + 
                                "s, Landing=" + parameters.LandingTime.ToString("F2") + 
                                "s, Penalty=" + penalty.ToString("F2"));
                Console.WriteLine("  Total cost: " + totalCost.ToString("F2"));
            }

            return totalCost;
        }

        private LaunchPhaseResult CalculateLaunchPhase(double launchFuel, bool verbose)
        {
            if (verbose)
            {
                Console.WriteLine("    Initializing launch phase with atmospheric drag...");
            }

            LaunchPhaseResult result = new LaunchPhaseResult();
            double timeStep = 0.1;
            double initialMass = rocket.Payload + rocket.TotalFuel;
            double mass = initialMass;
            double altitude = 0;
            double velocity = 0;
            double remainingLaunchFuel = launchFuel;
            double currentTime = 0;
            const double targetOrbitAltitude = PhysicalConstants.LowEarthOrbitAltitude;
            double minimumOrbitVelocity = CalculateCircularOrbitVelocity(PhysicalConstants.EarthRadius + targetOrbitAltitude, PhysicalConstants.EarthMass);

            if (verbose)
            {
                Console.WriteLine("    Initial mass: " + initialMass.ToString("F2") + " kg");
                Console.WriteLine("    Target orbit altitude: " + targetOrbitAltitude.ToString("F0") + " m");
                Console.WriteLine("    Minimum orbit velocity: " + minimumOrbitVelocity.ToString("F2") + " m/s");
            }

            int stepCount = 0;
            bool altitudeReached = false;
            bool velocityReached = false;
            
            while (remainingLaunchFuel > 0 && currentTime < 10000)
            {
                double radius = PhysicalConstants.EarthRadius + altitude;
                double gravity = PhysicalConstants.GravitationalConstant * PhysicalConstants.EarthMass / (radius * radius);
                
                double atmosphericDensity = CalculateAtmosphericDensity(altitude);
                double dragForce = 0.5 * atmosphericDensity * velocity * velocity * rocket.DragCoefficient * rocket.CrossSectionalArea;
                double dragAcceleration = dragForce / mass;
                
                double thrustAcceleration = rocket.Thrust / mass;
                double netAcceleration = thrustAcceleration - gravity - dragAcceleration;

                if (netAcceleration <= 0 && velocity < minimumOrbitVelocity * 0.5)
                {
                    if (verbose)
                    {
                        Console.WriteLine("    Insufficient acceleration: net=" + netAcceleration.ToString("F4") + " m/s², v=" + velocity.ToString("F2") + " m/s");
                    }
                    result.Success = false;
                    return result;
                }

                velocity += netAcceleration * timeStep;
                altitude += velocity * timeStep;

                double fuelConsumed = rocket.BurnRate * timeStep;
                remainingLaunchFuel -= fuelConsumed;
                mass -= fuelConsumed;
                
                currentTime += timeStep;
                stepCount++;

                altitudeReached = altitude >= targetOrbitAltitude;
                velocityReached = velocity >= minimumOrbitVelocity;

                if (verbose && stepCount % 500 == 0)
                {
                    double currentDeltaV = CalculateDeltaVFromTsiolkovsky(initialMass, mass);
                    Console.WriteLine("    Step " + stepCount + ": t=" + currentTime.ToString("F1") + 
                                    "s, alt=" + altitude.ToString("F0") + "m, v=" + velocity.ToString("F2") + 
                                    "m/s, mass=" + mass.ToString("F2") + "kg, drag=" + dragAcceleration.ToString("F4") + "m/s², deltaV=" + currentDeltaV.ToString("F2") + "m/s");
                }

                if (altitudeReached && velocityReached)
                {
                    if (verbose)
                    {
                        Console.WriteLine("    Orbit achieved: altitude=" + altitude.ToString("F0") + "m, velocity=" + velocity.ToString("F2") + "m/s");
                    }
                    break;
                }
            }

            result.Time = currentTime;
            result.Altitude = altitude;
            result.Velocity = velocity;
            result.DeltaV = CalculateDeltaVFromTsiolkovsky(initialMass, mass);
            
            double circularOrbitVelAtAchievedAlt = CalculateCircularOrbitVelocity(PhysicalConstants.EarthRadius + altitude, PhysicalConstants.EarthMass);
            double velocityRatio = velocity / circularOrbitVelAtAchievedAlt;
            result.Success = altitude >= targetOrbitAltitude * 0.8 && velocityRatio >= 0.88;

            if (verbose)
            {
                Console.WriteLine("    Launch phase complete: " + stepCount + " integration steps");
                Console.WriteLine("    Final altitude: " + altitude.ToString("F0") + " meters");
                Console.WriteLine("    Final velocity: " + velocity.ToString("F2") + " m/s");
                Console.WriteLine("    Total delta-V: " + result.DeltaV.ToString("F2") + " m/s");
            }

            return result;
        }

        private TransferOrbitResult CalculateHohmannTransfer(double initialAltitude, bool verbose)
        {
            TransferOrbitResult result = new TransferOrbitResult();
            
            double r1 = PhysicalConstants.EarthRadius + initialAltitude;
            double r2 = PhysicalConstants.EarthMoonDistance - PhysicalConstants.MoonRadius - PhysicalConstants.LunarOrbitAltitude;
            
            double semiMajorAxis = (r1 + r2) / 2.0;
            double mu = PhysicalConstants.GravitationalConstant * PhysicalConstants.EarthMass;
            
            double perigeeVelocity = Math.Sqrt(mu * (2.0 / r1 - 1.0 / semiMajorAxis));
            double apogeeVelocity = Math.Sqrt(mu * (2.0 / r2 - 1.0 / semiMajorAxis));
            double period = 2.0 * Math.PI * Math.Sqrt(Math.Pow(semiMajorAxis, 3) / mu);
            
            result.PerigeeVelocity = perigeeVelocity;
            result.ApogeeVelocity = apogeeVelocity;
            result.SemiMajorAxis = semiMajorAxis;
            result.Period = period;
            result.Success = true;

            if (verbose)
            {
                Console.WriteLine("    Hohmann transfer orbit calculated");
                Console.WriteLine("    Perigee radius: " + r1.ToString("F0") + " m");
                Console.WriteLine("    Apogee radius: " + r2.ToString("F0") + " m");
                Console.WriteLine("    Semi-major axis: " + semiMajorAxis.ToString("F0") + " m");
            }

            return result;
        }

        private LandingPhaseResult CalculateLunarLanding(double landingFuel, double initialOrbitVelocity, bool verbose)
        {
            if (verbose)
            {
                Console.WriteLine("    Calculating descent from lunar orbit...");
            }

            LandingPhaseResult result = new LandingPhaseResult();
            double timeStep = 0.1;
            double initialMass = rocket.Payload + landingFuel;
            double mass = initialMass;
            double altitude = PhysicalConstants.LunarOrbitAltitude;
            double velocity = initialOrbitVelocity;
            double remainingLandingFuel = landingFuel;
            double currentTime = 0;
            const double maxTime = 7200;
            const double targetDescentRate = 2.0;

            if (verbose)
            {
                Console.WriteLine("    Initial mass: " + initialMass.ToString("F2") + " kg");
                Console.WriteLine("    Initial altitude: " + altitude.ToString("F0") + " m");
                Console.WriteLine("    Initial velocity: " + velocity.ToString("F2") + " m/s");
                Console.WriteLine("    Target descent rate: " + targetDescentRate.ToString("F2") + " m/s");
            }

            int stepCount = 0;
            
            while (altitude > 0 && remainingLandingFuel > 1.0 && currentTime < maxTime)
            {
                double radius = PhysicalConstants.MoonRadius + altitude;
                double gravity = PhysicalConstants.GravitationalConstant * PhysicalConstants.MoonMass / (radius * radius);
                
                double desiredDeceleration;
                if (altitude > 50000)
                {
                    if (velocity > 1500)
                    {
                        desiredDeceleration = 6.0;
                    }
                    else
                    {
                        desiredDeceleration = 8.0;
                    }
                }
                else if (altitude > 20000)
                {
                    if (velocity > 1200)
                    {
                        desiredDeceleration = 8.0;
                    }
                    else
                    {
                        desiredDeceleration = 10.0;
                    }
                }
                else if (velocity > 1500)
                {
                    desiredDeceleration = 12.0;
                }
                else if (velocity > 1200)
                {
                    desiredDeceleration = 18.0;
                }
                else if (velocity > 800)
                {
                    desiredDeceleration = 35.0;
                }
                else if (velocity > 400)
                {
                    desiredDeceleration = 60.0;
                }
                else if (velocity > 100)
                {
                    desiredDeceleration = 75.0;
                }
                else if (velocity > 20)
                {
                    desiredDeceleration = 85.0;
                }
                else
                {
                    double timeToSurface = altitude / Math.Max(velocity, 1.0);
                    double requiredDecel = (velocity - rocket.LandingSpeedTarget) / Math.Max(timeToSurface, 0.1);
                    desiredDeceleration = Math.Min(requiredDecel, 35.0);
                }
                
                double requiredThrustForDeceleration = (desiredDeceleration + gravity) * mass;
                double actualThrust = Math.Min(requiredThrustForDeceleration, rocket.Thrust);
                double thrustAccel = actualThrust / mass;
                
                double netAcceleration = thrustAccel - gravity;

                if (netAcceleration <= 0 && velocity > 200.0 && altitude > 5000)
                {
                    if (verbose)
                    {
                        Console.WriteLine("    Insufficient thrust: net=" + netAcceleration.ToString("F4") + " m/s², v=" + velocity.ToString("F2") + " m/s, alt=" + altitude.ToString("F0") + "m");
                    }
                    result.Success = false;
                    return result;
                }

                velocity -= netAcceleration * timeStep;
                
                if (velocity < 0)
                {
                    velocity = 0;
                }
                
                double descentDistance = velocity * timeStep;
                altitude -= descentDistance;
                
                if (altitude < 0)
                {
                    altitude = 0;
                    if (velocity > rocket.LandingSpeedTarget + 1.0)
                    {
                        result.Success = false;
                        return result;
                    }
                    break;
                }

                double fuelConsumed = rocket.BurnRate * timeStep;
                remainingLandingFuel -= fuelConsumed;
                mass -= fuelConsumed;
                currentTime += timeStep;
                stepCount++;

                if (verbose && (stepCount % 100 == 0 || altitude < 20000))
                {
                    Console.WriteLine("    Step " + stepCount + ": t=" + currentTime.ToString("F1") + 
                                    "s, alt=" + altitude.ToString("F0") + "m, v=" + velocity.ToString("F2") + 
                                    "m/s, mass=" + mass.ToString("F2") + "kg, netAccel=" + netAcceleration.ToString("F4") + "m/s², fuel=" + remainingLandingFuel.ToString("F2") + "kg");
                }

                if (altitude <= 0.1 && Math.Abs(velocity - rocket.LandingSpeedTarget) <= 0.5)
                {
                    break;
                }
            }

            result.Time = currentTime;
            result.FinalVelocity = velocity;
            result.DeltaV = CalculateDeltaVFromTsiolkovsky(initialMass, mass);
            result.Success = altitude <= 200.0 && Math.Abs(velocity - rocket.LandingSpeedTarget) <= 10.0;

            if (verbose)
            {
                Console.WriteLine("    Landing phase complete: " + stepCount + " integration steps");
                Console.WriteLine("    Final altitude: " + altitude.ToString("F0") + " meters");
                Console.WriteLine("    Final velocity: " + velocity.ToString("F4") + " m/s");
                Console.WriteLine("    Descent delta-V: " + result.DeltaV.ToString("F2") + " m/s");
                Console.WriteLine("    Landing success: " + (result.Success ? "YES" : "NO"));
            }

            return result;
        }

        private double CalculateCircularOrbitVelocity(double radius, double centralMass)
        {
            double mu = PhysicalConstants.GravitationalConstant * centralMass;
            return Math.Sqrt(mu / radius);
        }

        private double CalculateAtmosphericDensity(double altitude)
        {
            if (altitude >= PhysicalConstants.KarmanLine)
            {
                return 0.0;
            }
            return PhysicalConstants.SeaLevelAtmosphericDensity * 
                   Math.Exp(-altitude / PhysicalConstants.AtmosphericScaleHeight);
        }

        private double CalculateDeltaVFromTsiolkovsky(double initialMass, double finalMass)
        {
            if (finalMass <= 0 || initialMass <= finalMass)
            {
                return 0;
            }
            return rocket.ExhaustVelocity * Math.Log(initialMass / finalMass);
        }

        private class LaunchPhaseResult
        {
            public bool Success { get; set; }
            public double Time { get; set; }
            public double Altitude { get; set; }
            public double Velocity { get; set; }
            public double DeltaV { get; set; }
        }

        private class TransferOrbitResult
        {
            public bool Success { get; set; }
            public double PerigeeVelocity { get; set; }
            public double ApogeeVelocity { get; set; }
            public double SemiMajorAxis { get; set; }
            public double Period { get; set; }
        }

        private class LandingPhaseResult
        {
            public bool Success { get; set; }
            public double Time { get; set; }
            public double FinalVelocity { get; set; }
            public double DeltaV { get; set; }
        }
    }
}
