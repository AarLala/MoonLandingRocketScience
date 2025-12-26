using System;

namespace RocketScienceCleaned
{
    public class FlightTracker
    {
        private readonly Rocket rocket;

        public FlightTracker(Rocket rocket)
        {
            this.rocket = rocket;
        }

        public void StartOptimization()
        {
            Console.WriteLine();
            Console.WriteLine("Rocket Flight Optimization System");
            Console.WriteLine("Gradient Descent Algorithm with Orbital Mechanics");
            Console.WriteLine();
            Console.WriteLine($"Rocket Configuration:");
            Console.WriteLine($"  Total Fuel: {rocket.TotalFuel:N0} kg");
            Console.WriteLine($"  Payload Mass: {rocket.Payload:N0} kg");
            Console.WriteLine($"  Thrust: {rocket.Thrust:N0} N");
            Console.WriteLine($"  Burn Rate: {rocket.BurnRate:F1} kg/s");
            Console.WriteLine($"  Exhaust Velocity: {rocket.ExhaustVelocity:F2} m/s");
            Console.WriteLine($"  Drag Coefficient: {rocket.DragCoefficient:F3}");
            Console.WriteLine($"  Cross-Sectional Area: {rocket.CrossSectionalArea:F2} m²");
            Console.WriteLine($"  Target Landing Speed: {rocket.LandingSpeedTarget:F2} m/s");
            Console.WriteLine();
            Console.WriteLine("Beginning optimization iterations...");
            Console.WriteLine();
        }

        public void UpdateOptimizationProgress(int iteration, double landingFuel, double cost, double previousCost, FlightParameters parameters)
        {
            double launchFuel = rocket.TotalFuel - landingFuel;
            double costChange = iteration > 0 && previousCost != double.MaxValue ? cost - previousCost : 0;

            Console.WriteLine($"ITERATION {iteration:D4} SUMMARY");
            Console.WriteLine();
            
            Console.WriteLine("Fuel Allocation Analysis");
            Console.WriteLine($"  Landing Fuel Reserved: {landingFuel,12:F2} kg ({landingFuel / rocket.TotalFuel * 100,6:F2}%)");
            Console.WriteLine($"  Launch Fuel Available:  {launchFuel,12:F2} kg ({launchFuel / rocket.TotalFuel * 100,6:F2}%)");
            Console.WriteLine($"  Remaining Mass:         {rocket.Payload + landingFuel,12:F2} kg");
            Console.WriteLine();

            Console.WriteLine("Launch Phase - Ascent to Orbit");
            Console.WriteLine($"  Launch Duration:       {parameters.LaunchTime,12:F2} seconds ({parameters.LaunchTime / 60,8:F2} minutes)");
            Console.WriteLine($"  Orbital Altitude:      {parameters.LaunchOrbitalAltitude,12:F0} meters ({parameters.LaunchOrbitalAltitude / 1000,8:F2} km)");
            Console.WriteLine($"  Circular Orbit Velocity: {parameters.CircularOrbitVelocity,12:F2} m/s");
            Console.WriteLine($"  Launch Delta-V:        {parameters.LaunchPhaseDeltaV,12:F2} m/s");
            Console.WriteLine($"  Fuel Consumed:         {parameters.LaunchFuel,12:F2} kg");
            Console.WriteLine();

            Console.WriteLine("Transfer Phase - Hohmann Transfer Orbit");
            Console.WriteLine($"  Transfer Period:       {parameters.TransferOrbitPeriod,12:F2} seconds ({parameters.TransferOrbitPeriod / 3600,8:F2} hours)");
            Console.WriteLine($"  Coast Duration:         {parameters.CoastTime,12:F2} seconds ({parameters.CoastTime / 3600,8:F2} hours)");
            Console.WriteLine($"  Semi-Major Axis:       {parameters.TransferOrbitSemiMajorAxis,12:F0} meters ({parameters.TransferOrbitSemiMajorAxis / 1000,8:F2} km)");
            Console.WriteLine($"  Perigee Velocity:      {parameters.TransferOrbitPerigeeVelocity,12:F2} m/s");
            Console.WriteLine($"  Apogee Velocity:       {parameters.TransferOrbitApogeeVelocity,12:F2} m/s");
            Console.WriteLine();

            Console.WriteLine("Lunar Orbit Insertion");
            Console.WriteLine($"  Lunar Orbit Altitude:  {parameters.LunarOrbitalAltitude,12:F0} meters ({parameters.LunarOrbitalAltitude / 1000,8:F2} km)");
            Console.WriteLine($"  Lunar Orbit Velocity:   {parameters.LunarOrbitVelocity,12:F2} m/s");
            Console.WriteLine($"  Insertion Delta-V:      {parameters.LunarOrbitInsertionDeltaV,12:F2} m/s");
            Console.WriteLine();

            Console.WriteLine("Landing Phase - Descent from Orbit");
            Console.WriteLine($"  Landing Duration:       {parameters.LandingTime,12:F2} seconds ({parameters.LandingTime / 60,8:F2} minutes)");
            Console.WriteLine($"  Descent Delta-V:        {parameters.LandingDescentDeltaV,12:F2} m/s");
            Console.WriteLine($"  Fuel Required:          {parameters.LandingFuel,12:F2} kg");
            Console.WriteLine($"  Final Landing Speed:    {parameters.FinalVelocity,12:F4} m/s");
            Console.WriteLine($"  Speed Deviation:        {Math.Abs(parameters.FinalVelocity - rocket.LandingSpeedTarget),12:F4} m/s");
            Console.WriteLine();

            Console.WriteLine("Delta-V Budget");
            Console.WriteLine($"  Launch Phase:           {parameters.LaunchPhaseDeltaV,12:F2} m/s");
            Console.WriteLine($"  Transfer Injection:    {parameters.TransferOrbitPerigeeVelocity - parameters.CircularOrbitVelocity,12:F2} m/s");
            Console.WriteLine($"  Lunar Orbit Insertion: {parameters.LunarOrbitInsertionDeltaV,12:F2} m/s");
            Console.WriteLine($"  Landing Descent:       {parameters.LandingDescentDeltaV,12:F2} m/s");
            Console.WriteLine($"  Total Mission Delta-V: {parameters.TotalDeltaV,12:F2} m/s");
            Console.WriteLine();

            Console.WriteLine("Optimization Metrics");
            Console.WriteLine($"  Total Flight Time:      {parameters.TotalFlightTime,12:F2} seconds ({parameters.TotalFlightTime / 3600,8:F2} hours)");
            Console.WriteLine($"  Cost Function Value:    {cost,12:F2}");
            Console.WriteLine($"  Landing Speed Penalty:  {parameters.CostPenalty,12:F2}");
            Console.WriteLine($"  Gradient:               {parameters.Gradient,12:F6}");
            Console.WriteLine($"  Cost Change:            {costChange,12:F6}");
            Console.WriteLine();

            Console.WriteLine("Phase Breakdown");
            Console.WriteLine($"  Launch:   {parameters.LaunchTime / parameters.TotalFlightTime * 100,6:F2}% of total flight time");
            Console.WriteLine($"  Coast:    {parameters.CoastTime / parameters.TotalFlightTime * 100,6:F2}% of total flight time");
            Console.WriteLine($"  Landing:  {parameters.LandingTime / parameters.TotalFlightTime * 100,6:F2}% of total flight time");
            Console.WriteLine();

            Console.WriteLine("Velocity Profile");
            Console.WriteLine($"  Earth Surface:          0.00 m/s");
            Console.WriteLine($"  Earth Orbit:           {parameters.CircularOrbitVelocity:F2} m/s");
            Console.WriteLine($"  Transfer Perigee:      {parameters.TransferOrbitPerigeeVelocity:F2} m/s");
            Console.WriteLine($"  Transfer Apogee:       {parameters.TransferOrbitApogeeVelocity:F2} m/s");
            Console.WriteLine($"  Lunar Orbit:           {parameters.LunarOrbitVelocity:F2} m/s");
            Console.WriteLine($"  Moon Surface:          {parameters.FinalVelocity:F4} m/s");
            Console.WriteLine();

            Console.WriteLine(new string('=', 80));
            Console.WriteLine();
        }

        public void LogInvalidSolution(int iteration, double landingFuel)
        {
            Console.WriteLine();
            Console.WriteLine($"WARNING: Invalid solution detected at iteration {iteration}");
            Console.WriteLine($"Landing fuel value {landingFuel:F2} kg is outside valid range");
            Console.WriteLine();
        }

        public void LogConvergence(int iteration)
        {
            Console.WriteLine();
            Console.WriteLine($"CONVERGENCE ACHIEVED");
            Console.WriteLine($"Optimization converged after {iteration} iterations");
            Console.WriteLine($"Cost function change below tolerance threshold");
            Console.WriteLine();
        }

        public void CompleteOptimization()
        {
            Console.WriteLine();
            Console.WriteLine("Optimization complete. Computing final trajectory parameters...");
            Console.WriteLine();
        }

        public void DisplayFinalResults(double landingFuel, FlightParameters parameters)
        {
            double launchFuel = rocket.TotalFuel - landingFuel;

            Console.WriteLine();
            Console.WriteLine("FINAL OPTIMIZED FLIGHT CONFIGURATION");
            Console.WriteLine();
            Console.WriteLine("Fuel Allocation Strategy");
            Console.WriteLine($"  Landing Fuel: {landingFuel:N0} kg ({landingFuel / rocket.TotalFuel * 100:F2}%)");
            Console.WriteLine($"  Launch Fuel:  {launchFuel:N0} kg ({launchFuel / rocket.TotalFuel * 100:F2}%)");
            Console.WriteLine($"  Total Fuel:   {rocket.TotalFuel:N0} kg");
            Console.WriteLine();

            Console.WriteLine("Flight Timeline");
            Console.WriteLine($"  Launch Phase:   {parameters.LaunchTime:F0} seconds ({parameters.LaunchTime / 60:F2} minutes)");
            Console.WriteLine($"  Transfer Phase: {parameters.CoastTime:F0} seconds ({parameters.CoastTime / 3600:F2} hours)");
            Console.WriteLine($"  Landing Phase:  {parameters.LandingTime:F0} seconds ({parameters.LandingTime / 60:F2} minutes)");
            Console.WriteLine($"  Total Duration: {parameters.TotalFlightTime:F0} seconds ({parameters.TotalFlightTime / 3600:F2} hours)");
            Console.WriteLine();

            Console.WriteLine("Orbital Mechanics");
            Console.WriteLine($"  Earth Orbit Altitude:  {parameters.LaunchOrbitalAltitude:F0} meters ({parameters.LaunchOrbitalAltitude / 1000:F2} km)");
            Console.WriteLine($"  Earth Orbit Velocity:  {parameters.CircularOrbitVelocity:F2} m/s");
            Console.WriteLine($"  Transfer Semi-Major Axis: {parameters.TransferOrbitSemiMajorAxis:F0} meters ({parameters.TransferOrbitSemiMajorAxis / 1000:F0} km)");
            Console.WriteLine($"  Transfer Period:       {parameters.TransferOrbitPeriod:F2} seconds ({parameters.TransferOrbitPeriod / 3600:F2} hours)");
            Console.WriteLine($"  Lunar Orbit Altitude:  {parameters.LunarOrbitalAltitude:F0} meters ({parameters.LunarOrbitalAltitude / 1000:F2} km)");
            Console.WriteLine($"  Lunar Orbit Velocity:  {parameters.LunarOrbitVelocity:F2} m/s");
            Console.WriteLine();

            Console.WriteLine("Delta-V Budget");
            Console.WriteLine($"  Launch to Orbit:       {parameters.LaunchPhaseDeltaV:F2} m/s");
            Console.WriteLine($"  Transfer Injection:    {parameters.TransferOrbitPerigeeVelocity - parameters.CircularOrbitVelocity:F2} m/s");
            Console.WriteLine($"  Lunar Orbit Insertion: {parameters.LunarOrbitInsertionDeltaV:F2} m/s");
            Console.WriteLine($"  Landing Descent:       {parameters.LandingDescentDeltaV:F2} m/s");
            Console.WriteLine($"  Total Mission Delta-V: {parameters.TotalDeltaV:F2} m/s");
            Console.WriteLine();

            Console.WriteLine("Velocity Achievements");
            Console.WriteLine($"  Earth Orbit Velocity:  {parameters.CircularOrbitVelocity:F2} m/s");
            Console.WriteLine($"  Transfer Perigee:       {parameters.TransferOrbitPerigeeVelocity:F2} m/s");
            Console.WriteLine($"  Transfer Apogee:        {parameters.TransferOrbitApogeeVelocity:F2} m/s");
            Console.WriteLine($"  Lunar Orbit Velocity:   {parameters.LunarOrbitVelocity:F2} m/s");
            Console.WriteLine($"  Final Landing Speed:    {parameters.FinalVelocity:F4} m/s");
            Console.WriteLine($"  Target Landing Speed:    {rocket.LandingSpeedTarget:F4} m/s");
            Console.WriteLine($"  Landing Accuracy:       {Math.Abs(parameters.FinalVelocity - rocket.LandingSpeedTarget):F4} m/s deviation");
            Console.WriteLine();

            Console.WriteLine("Performance Metrics");
            Console.WriteLine($"  Total Flight Time:      {parameters.TotalFlightTime:F2} seconds");
            Console.WriteLine($"  Cost Function Value:    {parameters.Cost:F2}");
            Console.WriteLine($"  Landing Speed Penalty:  {parameters.CostPenalty:F2}");
            Console.WriteLine($"  Specific Impulse:       {rocket.ExhaustVelocity / 9.81:F1} seconds");
            Console.WriteLine();

            Console.WriteLine("Mission Summary");
            Console.WriteLine($"  Starting Mass:          {rocket.Payload + rocket.TotalFuel:N0} kg");
            Console.WriteLine($"  Payload Mass:           {rocket.Payload:N0} kg");
            double speedDeviation = Math.Abs(parameters.FinalVelocity - rocket.LandingSpeedTarget);
            bool missionSuccess = parameters.FinalVelocity <= rocket.LandingSpeedTarget + 0.5;
            Console.WriteLine($"  Mission Success:        {(missionSuccess ? "YES" : "NO")}");
            Console.WriteLine();
        }
    }
}
