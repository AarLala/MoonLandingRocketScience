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

            double baseCost = 0.0;
            double penalty = 0.0;
            const double w_launch = 1e6;
            const double w_loi = 1e6;
            const double w_land = 1e6;
            const double w_alt = 1e2;

            if (landingFuel <= 0 || landingFuel >= rocket.TotalFuel)
            {
                double violation = landingFuel <= 0 ? Math.Abs(landingFuel) : Math.Abs(landingFuel - rocket.TotalFuel);
                penalty += w_launch * violation * violation;
                parameters.Cost = baseCost + penalty;
                return baseCost + penalty;
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
                double targetOrbitVelocity = CalculateCircularOrbitVelocity(PhysicalConstants.EarthRadius + PhysicalConstants.LowEarthOrbitAltitude, PhysicalConstants.EarthMass);
                double dvRequired = targetOrbitVelocity;
                double dvAchieved = launchResult.DeltaV;
                double dvDeficit = Math.Max(0, dvRequired - dvAchieved);
                penalty += w_launch * dvDeficit * dvDeficit;
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
                double targetRatio = 0.88;
                double ratioDeficit = Math.Max(0, targetRatio - velocityRatio);
                double altDeficit = Math.Max(0, minRequiredAltitude - parameters.LaunchOrbitalAltitude);
                penalty += w_launch * (ratioDeficit * circularOrbitVelocityAtAltitude) * (ratioDeficit * circularOrbitVelocityAtAltitude);
                penalty += w_launch * altDeficit * altDeficit / 1e6;
            }

            if (verbose)
            {
                Console.WriteLine("  Calculating Hohmann transfer orbit...");
            }

            TransferOrbitResult transferResult = CalculateHohmannTransfer(parameters.LaunchOrbitalAltitude, verbose);
            
            if (!transferResult.Success)
            {
                penalty += w_loi * 1e6;
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
                Console.WriteLine("  Transfer orbit period: " + (transferResult.Period / 3600.0).ToString("F2") + " hours");
                Console.WriteLine("  Transfer delta-V required (TLI): " + transferDeltaV.ToString("F2") + " m/s");
                Console.WriteLine("  Expected range for TLI: ~3,100-3,300 m/s");
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
                Console.WriteLine("  Lunar orbit altitude: " + PhysicalConstants.LunarOrbitAltitude.ToString("F0") + " m");
                Console.WriteLine("  Lunar orbit velocity (calculated): " + lunarOrbitVelocity.ToString("F2") + " m/s");
                Console.WriteLine("  Expected range for 100 km lunar orbit: ~1,600-1,700 m/s");
                Console.WriteLine("  Transfer apogee velocity: " + transferResult.ApogeeVelocity.ToString("F2") + " m/s");
                Console.WriteLine("  Lunar orbit insertion delta-V: " + lunarOrbitInsertionDeltaV.ToString("F2") + " m/s");
                Console.WriteLine("  Expected range for LOI: ~800-1,000 m/s");
            }

            if (verbose)
            {
                Console.WriteLine("  Calculating landing descent from lunar orbit...");
            }

            LandingPhaseResult landingResult = CalculateLunarLanding(landingFuel, lunarOrbitVelocity, verbose);
            
            parameters.LandingTime = landingResult.Time;
            parameters.FinalVelocity = landingResult.FinalVelocity;
            parameters.LandingDescentDeltaV = landingResult.DeltaV;
            
            parameters.TotalDeltaV = parameters.LaunchPhaseDeltaV + transferDeltaV + lunarOrbitInsertionDeltaV + landingResult.DeltaV;
            
            if (!landingResult.Success)
            {
                if (verbose)
                {
                    if (landingResult.FuelDeficit > 0)
                    {
                        Console.WriteLine("  Landing phase failed: Fuel exhausted, deficit: " + landingResult.FuelDeficit.ToString("F2") + " kg");
                        Console.WriteLine("  Failure at altitude: " + landingResult.AltitudeAtFailure.ToString("F0") + " m, velocity: " + landingResult.VelocityAtFailure.ToString("F2") + " m/s");
                    }
                    else
                    {
                        Console.WriteLine("  Landing phase failed: Insufficient fuel or thrust");
                        Console.WriteLine("  Failure at altitude: " + landingResult.AltitudeAtFailure.ToString("F0") + " m, velocity: " + landingResult.VelocityAtFailure.ToString("F2") + " m/s");
                    }
                }
                
                double altitudeRemaining = landingResult.AltitudeAtFailure;
                double vFinal = Math.Abs(landingResult.VelocityAtFailure);
                double vTarget = rocket.LandingSpeedTarget;
                
                penalty += w_land * (vFinal - vTarget) * (vFinal - vTarget);
                penalty += w_alt * altitudeRemaining * altitudeRemaining;
                
                parameters.Cost = baseCost + penalty;
                return baseCost + penalty;
            }

            if (verbose)
            {
                Console.WriteLine("  Landing descent delta-V: " + landingResult.DeltaV.ToString("F2") + " m/s");
                Console.WriteLine("  Final landing velocity: " + landingResult.FinalVelocity.ToString("F4") + " m/s");
            }

            baseCost = parameters.TotalDeltaV;
            double speedDeviation = Math.Abs(parameters.FinalVelocity - rocket.LandingSpeedTarget);
            double totalFuelUsed = parameters.LaunchFuel + landingFuel;
            
            double landingSpeedPenalty = 0.0;
            if (speedDeviation > 0.1)
            {
                double penaltyWeight = 1e7;
                if (parameters.FinalVelocity > rocket.LandingSpeedTarget + 0.5)
                {
                    penaltyWeight = 1e9;
                }
                else if (parameters.FinalVelocity > rocket.LandingSpeedTarget)
                {
                    penaltyWeight = 5e7;
                }
                landingSpeedPenalty = penaltyWeight * speedDeviation * speedDeviation;
                
                if (parameters.FinalVelocity > rocket.LandingSpeedTarget + 0.5)
                {
                    double excessSpeed = parameters.FinalVelocity - (rocket.LandingSpeedTarget + 0.5);
                    landingSpeedPenalty += excessSpeed * excessSpeed * 1e9;
                }
            }
            
            baseCost += landingSpeedPenalty + totalFuelUsed;
            parameters.CostPenalty = landingSpeedPenalty;
            
            parameters.Cost = baseCost + penalty;
            return baseCost + penalty;
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
            
            // Launch delta-V should be the actual velocity achieved (includes gravity/drag losses)
            // This is more accurate than using Tsiolkovsky which gives theoretical maximum
            // For Earth launch to LEO, this should be ~9.3-10.0 km/s including losses
            result.DeltaV = velocity;
            
            double circularOrbitVelAtAchievedAlt = CalculateCircularOrbitVelocity(PhysicalConstants.EarthRadius + altitude, PhysicalConstants.EarthMass);
            double velocityRatio = velocity / circularOrbitVelAtAchievedAlt;
            
            // Validation: check that achieved velocity is reasonable for the altitude
            if (verbose)
            {
                Console.WriteLine("    Validation: Circular orbit velocity at achieved altitude: " + circularOrbitVelAtAchievedAlt.ToString("F2") + " m/s");
                Console.WriteLine("    Validation: Achieved velocity: " + velocity.ToString("F2") + " m/s");
                Console.WriteLine("    Validation: Velocity ratio: " + velocityRatio.ToString("F3"));
            }
            
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
                Console.WriteLine("    Perigee radius (LEO): " + (r1 / 1000.0).ToString("F2") + " km");
                Console.WriteLine("    Apogee radius (Moon distance): " + (r2 / 1000.0).ToString("F2") + " km");
                Console.WriteLine("    Semi-major axis: " + (semiMajorAxis / 1000.0).ToString("F2") + " km");
                Console.WriteLine("    Perigee velocity: " + perigeeVelocity.ToString("F2") + " m/s");
                Console.WriteLine("    Apogee velocity: " + apogeeVelocity.ToString("F2") + " m/s");
                Console.WriteLine("    Transfer period: " + (period / 3600.0).ToString("F2") + " hours");
            }

            return result;
        }

        private LandingPhaseResult CalculateLunarLanding(double landingFuel, double initialOrbitVelocity, bool verbose)
        {
            LandingPhaseResult result = new LandingPhaseResult();
            double timeStep = 0.1;
            double initialMass = rocket.Payload + landingFuel;
            double mass = initialMass;
            double altitude = PhysicalConstants.LunarOrbitAltitude;
            double remainingLandingFuel = landingFuel;
            double currentTime = 0;
            const double maxTime = 7200;
            
            double vHorizontal = initialOrbitVelocity;
            int brakeSteps = 0;
            
            while (vHorizontal > 5.0 && remainingLandingFuel > 0 && currentTime < maxTime)
            {
                if (remainingLandingFuel <= 0)
                {
                    result.Success = false;
                    result.FuelDeficit = -remainingLandingFuel;
                    result.AltitudeAtFailure = altitude;
                    result.VelocityAtFailure = vHorizontal;
                    result.Time = currentTime;
                    result.FinalVelocity = vHorizontal;
                    result.DeltaV = initialOrbitVelocity - vHorizontal;
                    return result;
                }
                
                double radius = PhysicalConstants.MoonRadius + altitude;
                double gravity = PhysicalConstants.GravitationalConstant * PhysicalConstants.MoonMass / (radius * radius);
                
                double thrustAccel = rocket.Thrust / mass;
                double maxFuelAvailable = remainingLandingFuel;
                double maxThrustFromFuel = (maxFuelAvailable / timeStep) * rocket.ExhaustVelocity * rocket.BurnRate / rocket.Thrust;
                if (maxThrustFromFuel < rocket.Thrust)
                {
                    thrustAccel = maxThrustFromFuel / mass;
                }
                
                vHorizontal -= thrustAccel * timeStep;
                
                double fuelConsumed = Math.Min(rocket.BurnRate * timeStep, remainingLandingFuel);
                remainingLandingFuel -= fuelConsumed;
                mass -= fuelConsumed;
                currentTime += timeStep;
                brakeSteps++;
            }
            
            double velocity = 0.0;
            int descentSteps = 0;
            double altitudeAtFailure = altitude;
            double velocityAtFailure = velocity;
            bool fuelExhausted = false;
            double previousVelocity = velocity;
            
            while (altitude > 0 && currentTime < maxTime)
            {
                double radius = PhysicalConstants.MoonRadius + altitude;
                double gravity = PhysicalConstants.GravitationalConstant * PhysicalConstants.MoonMass / (radius * radius);
                
                double vTarget = 0.0;
                double aCmd = 0.0;
                double actualThrust = 0.0;
                
                if (altitude > 20000.0)
                {
                    actualThrust = 0.0;
                }
                else if (altitude > 2000.0)
                {
                    vTarget = -50.0;
                    double velocityError = vTarget - velocity;
                    double velocityDerivative = (velocity - previousVelocity) / timeStep;
                    double kp = 1.8;
                    double kd = 0.3;
                    aCmd = (kp * velocityError - kd * velocityDerivative) / timeStep;
                    double requiredThrust = mass * (aCmd + gravity);
                    actualThrust = Math.Max(0.0, Math.Min(requiredThrust, rocket.Thrust));
                }
                else
                {
                    double targetSpeed = rocket.LandingSpeedTarget;
                    if (altitude > 1500.0)
                    {
                        targetSpeed = 20.0 + (altitude - 1500.0) / 500.0 * 30.0;
                    }
                    else if (altitude > 800.0)
                    {
                        targetSpeed = 12.0 + (altitude - 800.0) / 700.0 * 8.0;
                    }
                    else if (altitude > 400.0)
                    {
                        targetSpeed = 7.0 + (altitude - 400.0) / 400.0 * 5.0;
                    }
                    else if (altitude > 150.0)
                    {
                        targetSpeed = 4.5 + (altitude - 150.0) / 250.0 * 2.5;
                    }
                    else if (altitude > 30.0)
                    {
                        targetSpeed = rocket.LandingSpeedTarget + (altitude - 30.0) / 120.0 * 1.5;
                    }
                    else if (altitude > 10.0)
                    {
                        targetSpeed = rocket.LandingSpeedTarget + (altitude - 10.0) / 20.0 * 0.5;
                    }
                    else
                    {
                        targetSpeed = rocket.LandingSpeedTarget;
                    }
                    
                    vTarget = -targetSpeed;
                    double velocityError = vTarget - velocity;
                    double velocityDerivative = (velocity - previousVelocity) / timeStep;
                    
                    double kp = 4.0;
                    double kd = 0.5;
                    if (altitude < 50.0)
                    {
                        kp = 8.0;
                        kd = 1.0;
                    }
                    else if (altitude < 150.0)
                    {
                        kp = 6.0;
                        kd = 0.8;
                    }
                    
                    aCmd = (kp * velocityError - kd * velocityDerivative) / timeStep;
                    double requiredThrust = mass * (aCmd + gravity);
                    actualThrust = Math.Max(0.0, Math.Min(requiredThrust, rocket.Thrust));
                }
                
                previousVelocity = velocity;
                
                if (actualThrust > 0 && remainingLandingFuel > 0)
                {
                    double maxFuelAvailable = remainingLandingFuel;
                    double maxThrustFromFuel = (maxFuelAvailable / timeStep) * rocket.ExhaustVelocity * rocket.BurnRate / rocket.Thrust;
                    if (maxThrustFromFuel < actualThrust)
                    {
                        actualThrust = maxThrustFromFuel;
                    }
                    
                    double fuelConsumed = Math.Min(rocket.BurnRate * timeStep, remainingLandingFuel);
                    remainingLandingFuel -= fuelConsumed;
                    mass -= fuelConsumed;
                    
                    if (remainingLandingFuel <= 0)
                    {
                        fuelExhausted = true;
                        altitudeAtFailure = altitude;
                        velocityAtFailure = velocity;
                    }
                }
                
                double thrustAccel = actualThrust / mass;
                double netAcceleration = thrustAccel - gravity;
                velocity += netAcceleration * timeStep;
                altitude += velocity * timeStep;
                
                if (altitude <= 0)
                {
                    altitude = 0;
                    double landingSpeed = Math.Abs(velocity);
                    if (landingSpeed > rocket.LandingSpeedTarget + 0.5)
                    {
                        result.Success = false;
                        result.FuelDeficit = remainingLandingFuel < 0 ? -remainingLandingFuel : 0;
                        result.AltitudeAtFailure = altitude;
                        result.VelocityAtFailure = velocity;
                        result.Time = currentTime;
                        result.FinalVelocity = landingSpeed;
                        result.DeltaV = initialOrbitVelocity + landingSpeed;
                        return result;
                    }
                    break;
                }
                
                if (fuelExhausted)
                {
                    result.Success = false;
                    result.FuelDeficit = -remainingLandingFuel;
                    result.AltitudeAtFailure = altitudeAtFailure;
                    result.VelocityAtFailure = velocityAtFailure;
                    result.Time = currentTime;
                    result.FinalVelocity = Math.Abs(velocityAtFailure);
                    result.DeltaV = initialOrbitVelocity + Math.Abs(velocityAtFailure);
                    return result;
                }
                
                currentTime += timeStep;
                descentSteps++;
            }
            
            result.Time = currentTime;
            double actualLandingSpeed = Math.Abs(velocity);
            result.FinalVelocity = actualLandingSpeed;
            result.DeltaV = initialOrbitVelocity + actualLandingSpeed;
            result.Success = altitude <= 200.0 && actualLandingSpeed <= rocket.LandingSpeedTarget + 0.5;
            
            if (!result.Success)
            {
                result.AltitudeAtFailure = altitude;
                result.VelocityAtFailure = velocity;
                result.FuelDeficit = remainingLandingFuel < 0 ? -remainingLandingFuel : 0;
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
            public double FuelDeficit { get; set; }
            public double AltitudeAtFailure { get; set; }
            public double VelocityAtFailure { get; set; }
        }
    }
}
