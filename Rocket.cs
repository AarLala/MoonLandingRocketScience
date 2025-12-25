namespace RocketScienceCleaned
{
    public class Rocket
    {
        public double Thrust { get; }
        public double BurnRate { get; }
        public double ExhaustVelocity { get; }
        public double TotalFuel { get; }
        public double Payload { get; }
        public double LandingSpeedTarget { get; }
        public double DragCoefficient { get; }
        public double CrossSectionalArea { get; }

        public Rocket(double thrust, double burnRate, double totalFuel, double payload, double landingSpeedTarget, double dragCoefficient, double crossSectionalArea)
        {
            Thrust = thrust;
            BurnRate = burnRate;
            ExhaustVelocity = thrust / burnRate;
            TotalFuel = totalFuel;
            Payload = payload;
            LandingSpeedTarget = landingSpeedTarget;
            DragCoefficient = dragCoefficient;
            CrossSectionalArea = crossSectionalArea;
        }
    }
}
