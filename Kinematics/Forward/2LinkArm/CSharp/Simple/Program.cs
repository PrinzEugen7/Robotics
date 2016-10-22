using System;
using System.Collections.Generic;
using System.Text;

namespace Test
{
     class FK
    {
        static void Main() {
			forwardKinematics(0.5, 0.6, 30, 40);
   			double x = forwardKinematics.Item1;
			double y = forwardKinematics.Item2;
			Console.WriteLine(x);
  		}

		public Tuple<double, double> forwardKinematics(double l1, double l2, double deg1, double deg2)
		{
			// 度をラジアンに変換
    		double rad1 = degToRad(deg1);
    		double rad2 = degToRad(deg2);
    		// 手先位置を順運動学の式で算出
			double x = Math.cos(rad1)+Math.cos(rad1+rad2);
			double y = Math.sin(rad1)+Math.sin(rad1+rad2);
 
 			return Tuple.Create(res1, res2);
		}
		public static double degToRad (double degrees)
		{
    		double radians = (Math.PI / 180) * degrees;
			return (radians);
		}

    }

}
