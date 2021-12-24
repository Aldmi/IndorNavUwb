namespace Domain.Core.Algoritms.TDoA.Model;

public struct TDOABaseline
{
    public double X1;
    public double Y1;
    public double Z1;
    public double X2;
    public double Y2;
    public double Z2;
    public double PRD; // Pseudorange difference

    public TDOABaseline(double x1, double y1, double z1, double x2, double y2, double z2, double prd)
    {
        X1 = x1;
        Y1 = y1;
        Z1 = z1;
        X2 = x2;
        Y2 = y2;
        Z2 = z2;
        PRD = prd;
    }


    /// <summary>
    /// Build baselines on collection of base points (combinatoric)
    /// </summary>
    /// <param name="bases">Base points</param>
    /// <param name="velocity">Signal propagation velocity in m/s</param>
    /// <returns></returns>
    public static TDOABaseline[] BuildBaseLines(TOABasePoint[] bases, double velocity)
    {
        List<TDOABaseline> result = new List<TDOABaseline>();

        for (int i = 0; i < bases.Length - 1; i++)
        for (int j = i + 1; j < bases.Length; j++)
        {
            result.Add(new TDOABaseline(bases[i].X, bases[i].Y, bases[i].Z,
                bases[j].X, bases[j].Y, bases[j].Z,
                (bases[i].D - bases[j].D) * velocity));
        }

        return result.ToArray();
    }
}