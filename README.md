# Rocket Flight Optimization

This project calculates the optimal fuel allocation for a rocket traveling from Earth to the Moon. It uses gradient descent with sigmoid constraint mapping to find the best balance between launch fuel and landing fuel, ensuring the rocket can both escape Earth's gravity and safely land on the Moon.

## What It Does

The program simulates a rocket flight with three main phases: launch from Earth, coasting through space, and landing on the Moon. It determines how much fuel to save for landing while using enough fuel during launch to reach the required velocity. The optimization process uses an unconstrained variable mapped through a sigmoid function to smoothly explore the feasible region.

## Project Structure

**PhysicalConstants.cs** - Stores physical constants like Earth's mass, Moon's radius, and gravitational constant.

**Rocket.cs** - Represents the rocket with properties like thrust, burn rate, total fuel, payload weight, and target landing speed.

**FlightParameters.cs** - Data container for flight calculation results including launch time, velocities, altitudes, and total cost.

**TrajectoryCalculator.cs** - Performs physics calculations for launch, coasting phase, and landing burn. Computes the cost function that the optimizer minimizes.

**Optimizer.cs** - Implements gradient descent with sigmoid constraint mapping. Uses an unconstrained internal variable that smoothly maps to the feasible fuel allocation range.

**FlightTracker.cs** - Handles console output showing progress during optimization and final flight configuration.

**Program.cs** - Main entry point that creates objects, runs optimization, and displays results.

## How to Run

Make sure you have the .NET SDK installed (this project targets .NET 9.0). Then run:

```
dotnet run
```

The program will optimize the fuel allocation and display progress. Once it converges, you'll get a detailed breakdown of the final flight configuration.

## How It Works

The optimization process uses gradient descent with a sigmoid constraint mapping. An unconstrained variable `y` is mapped to the feasible fuel ratio range [0.30, 0.60] using a sigmoid function. This allows the optimizer to smoothly explore the entire feasible region without hitting hard boundaries.

For each iteration, the optimizer:
1. Calculates the gradient in the unconstrained space
2. Normalizes the gradient to prevent numerical issues
3. Performs a line search to find an acceptable step
4. Updates the fuel allocation based on the best step found

The trajectory calculation accounts for variable gravity during launch, atmospheric drag, Hohmann transfer orbit mechanics, and the physics of deceleration during the Moon landing phase.

## Output

When you run the program, you'll see:
- Fuel allocation between launch and landing phases
- Duration of each flight phase (launch, coast, landing)
- Velocity profile showing orbital velocities and final landing speed
- Altitude milestones for Earth departure and Moon landing
- Total mission delta-V budget
- Mission success status (YES or NO based on landing speed)

All optimization data is saved to a CSV file on your desktop for further analysis.
