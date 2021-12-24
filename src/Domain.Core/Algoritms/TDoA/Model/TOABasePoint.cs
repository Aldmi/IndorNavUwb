namespace Domain.Core.Algoritms.TDoA.Model;

public struct TOABasePoint
{
    public double X;
    public double Y;
    public double Z;
    public double D; // Distance or pseudorange

    public TOABasePoint(double x, double y, double z, double d)
    {
        X = x;
        Y = y;
        Z = z;
        D = d;
    }
}