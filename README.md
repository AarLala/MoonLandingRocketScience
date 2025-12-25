# Rocket Flight Optimization

This project calculates the optimal fuel allocation for a rocket traveling from Earth to the Moon. It uses gradient descent to find the best balance between launch fuel and landing fuel, ensuring the rocket can both escape Earth's gravity and safely land on the Moon.

## What It Does

The program simulates a rocket flight with three main phases: launch from Earth, coasting through space, and landing on the Moon. It figures out how much fuel to save for landing while using enough fuel during launch to reach the required velocity. The optimization process runs through multiple iterations, tracking progress and displaying results as it finds the best solution.

## Project Structure

The code is organized into several classes, each handling a specific part of the calculation:

**PhysicalConstants.cs** - Stores all the physical constants we need, like Earth's mass, the Moon's radius, and the gravitational constant. These values are used throughout the calculations.

**Rocket.cs** - Represents the rocket itself. It holds information about the rocket's thrust, how fast it burns fuel, how much total fuel it has, the payload weight, and the target landing speed.

**FlightParameters.cs** - A simple data container that holds all the results from flight calculations. Things like launch time, coast time, velocities, altitudes, and the total cost of the flight.

**TrajectoryCalculator.cs** - Does the heavy lifting when it comes to physics calculations. It figures out how the rocket moves during launch, calculates the coasting phase, and determines what happens during the landing burn. It also computes the cost function that the optimizer tries to minimize.

**Optimizer.cs** - Implements the gradient descent algorithm. It starts with an initial guess for how much fuel to save for landing, then iteratively improves that guess by calculating gradients and adjusting the fuel allocation. It works with the trajectory calculator to evaluate each potential solution.

**FlightTracker.cs** - Handles all the console output. It shows progress during optimization, displays intermediate results, and presents the final flight configuration in a readable format.

**Program.cs** - The main entry point. It creates all the necessary objects, runs the optimization, and displays the final results.

## How to Run

Make sure you have the .NET SDK installed (this project targets .NET 9.0). Then run:

```
dotnet run
```

The program will start optimizing the fuel allocation and show you progress updates every 10 iterations. You'll see the landing fuel allocation, total flight time, injection velocity, and final landing speed as the algorithm searches for the best solution. Once it converges, you'll get a detailed breakdown of the final flight configuration.

## How It Works

The optimization process starts by splitting the total fuel roughly in half between launch and landing. Then it uses gradient descent to refine this allocation. For each iteration, it calculates the flight trajectory with the current fuel split, evaluates how good that solution is (considering flight time and landing speed), and then adjusts the fuel allocation based on the gradient.

The trajectory calculation accounts for variable gravity during launch, a realistic injection velocity cap, and the physics of deceleration during the Moon landing phase. The cost function penalizes solutions that result in landing speeds higher than the target, encouraging safe landings.

## Output

When you run the program, you'll see real-time updates showing the optimization progress. The final output includes:

- Fuel allocation between launch and landing phases
- Duration of each flight phase (launch, coast, landing)
- Velocity profile showing injection velocity and final landing speed
- Altitude milestones for Earth departure and Moon landing burn start
- Total flight time

All of this information helps you understand how the rocket will perform with the optimized fuel allocation.

