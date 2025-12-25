namespace RocketScienceCleaned
{
    public class FlightParameters
    {
        public double LaunchTime { get; set; }
        public double CoastTime { get; set; }
        public double LandingTime { get; set; }
        public double FinalVelocity { get; set; }
        public double LaunchStopAltitude { get; set; }
        public double Cost { get; set; }
        public double LaunchFuel { get; set; }
        public double LandingFuel { get; set; }
        public double CostPenalty { get; set; }
        public double Gradient { get; set; }
        public double CircularOrbitVelocity { get; set; }
        public double TransferOrbitPerigeeVelocity { get; set; }
        public double TransferOrbitApogeeVelocity { get; set; }
        public double LunarOrbitInsertionDeltaV { get; set; }
        public double LunarOrbitVelocity { get; set; }
        public double LandingDescentDeltaV { get; set; }
        public double TotalDeltaV { get; set; }
        public double LaunchPhaseDeltaV { get; set; }
        public double TransferOrbitPeriod { get; set; }
        public double TransferOrbitSemiMajorAxis { get; set; }
        public double LaunchOrbitalAltitude { get; set; }
        public double LunarOrbitalAltitude { get; set; }

        public double TotalFlightTime => LaunchTime + CoastTime + LandingTime;
    }
}
