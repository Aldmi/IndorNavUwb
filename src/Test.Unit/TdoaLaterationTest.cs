using Domain.Core.Algoritms.TDoA;
using Domain.Core.Algoritms.TDoA.Model;
using Utils;
using Xunit;

namespace Test.Unit
{
    public class TdoaLaterationTest
    {
        [Fact]
        public void TDOA_Locate2D_Test()
        {
            //arrange
            var start_bases_z = 1.5;
            var basePoints = new TOABasePoint[]
            {
                new(0.0, 5.0, start_bases_z, (2.0 * 2)/PhysicalConstants.SpeedOfLight),
                new(0.0, 15.0, start_bases_z, (3.7 * 2)/PhysicalConstants.SpeedOfLight),
                new(10.0, 15.0, start_bases_z, (4.9 * 2)/PhysicalConstants.SpeedOfLight),
                new(10.0, 5.0, start_bases_z, (3.9 * 2)/PhysicalConstants.SpeedOfLight)
            };

            //tOABasePoints[0] = new TOABasePoint(0.0, 5.0, start_bases_z, (2.0 * 2) / velocity);
            //tOABasePoints[1] = new TOABasePoint(0.0, 15.0, start_bases_z, (3.7 * 2) / velocity);
            //tOABasePoints[2] = new TOABasePoint(10.0, 15.0, start_bases_z, (4.9 * 2) / velocity);
            //tOABasePoints[3] = new TOABasePoint(10.0, 5.0, start_bases_z, (3.9 * 2) / velocity);



            //act
           var res= TdoaLateration.TDOA_Locate2D_WithOptimalArgs(basePoints);

            //assert
        }
    }
}