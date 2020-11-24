using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ARAPTriangle : MonoBehaviour
{ 
    public int[] vertex_id;
    public Vector2[] local_xy;
    public float scale_factor_divisor;
    public Matrix mF;
    public Matrix mC;

	public ARAPTriangle()
    {
		vertex_id = new int[3];
		local_xy = new Vector2[3];
    }

	public void SetVertexID(int a,int b,int c)
    {
		vertex_id[0] = a;
		vertex_id[1] = b;
		vertex_id[2] = c;
    }

	public void SetLocalXY(Vector2 a, Vector2 b, Vector2 c)
    {
		local_xy[0] = a;
		local_xy[1] = b;
		local_xy[2] = c;
    }

	public void PreComputeScaleAdjustmentMatrixFC()
	{
		float x01 = local_xy[2].x;
		float y01 = local_xy[2].y;
		float x12 = local_xy[0].x;
		float y12 = local_xy[0].y;
		float x20 = local_xy[1].x;
		float y20 = local_xy[1].y;

		float f1 = (1 - x01) * x12 + y01 * y12;
		float f2 = -(x12 * y01) + (1 - x01) * y12;
		float f3 = 1 - x01 + x01 * x20 - y01 * y20;
		float f4 = -y01 + x20 * y01 + x01 * y20;
		float f5 = x12 * y01 + (-1 + x01) * y12;
		float f6 = y01 - x20 * y01 - x01 * y20;
		float f7 = 1 + (-1 + x01) * x12 - y01 * y12;
		float f8 = x01 - x01 * x20 + y01 * y20;

		mF =Matrix.ZeroMatrix(4,4);
		mF[0, 0] = 4 - 8 * x01 + 4 * Mathf.Pow(x01, 2) + 2 * Mathf.Pow(x12, 2) - 4 * x01 * Mathf.Pow(x12, 2) + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(x12, 2) + 4 * x01 * x20 - 4 * Mathf.Pow(x01, 2) * x20 + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(x20, 2) +
			4 * Mathf.Pow(y01, 2) + 2 * Mathf.Pow(x12, 2) * Mathf.Pow(y01, 2) - 4 * x20 * Mathf.Pow(y01, 2) + 2 * Mathf.Pow(x20, 2) * Mathf.Pow(y01, 2) + 2 * Mathf.Pow(y12, 2) - 4 * x01 * Mathf.Pow(y12, 2) + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(y12, 2) +
			2 * Mathf.Pow(y01, 2) * Mathf.Pow(y12, 2) - 4 * y01 * y20 + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(y20, 2) + 2 * Mathf.Pow(y01, 2) * Mathf.Pow(y20, 2);
		mF[0, 1] = 2 * f2 * f1 + 2 * f5 * f1 +
			2 * f6 * f3 + 2 * f4 * f3;
		mF[0, 2] = 2 * (1 - x01) * x01 - 2 * Mathf.Pow(y01, 2) + 2 * f2 * f5 + 2 * f7 * f1 +
			2 * f6 * f4 + 2 * f3 * f8;
		mF[0, 3] = -2 * (1 - x01) * y01 - 2 * x01 * y01 + 2 * f2 * f7 + 2 * f2 * f1 +
			2 * f4 * f3 + 2 * f4 * f8;
		mF[1, 0] = 2 * f2 * f1 + 2 * f5 * f1 +
			2 * f6 * f3 + 2 * f4 * f3;
		mF[1, 1] = 4 - 8 * x01 + 4 * Mathf.Pow(x01, 2) + 2 * Mathf.Pow(x12, 2) - 4 * x01 * Mathf.Pow(x12, 2) + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(x12, 2) + 4 * x01 * x20 - 4 * Mathf.Pow(x01, 2) * x20 + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(x20, 2) +
			4 * Mathf.Pow(y01, 2) + 2 * Mathf.Pow(x12, 2) * Mathf.Pow(y01, 2) - 4 * x20 * Mathf.Pow(y01, 2) + 2 * Mathf.Pow(x20, 2) * Mathf.Pow(y01, 2) + 2 * Mathf.Pow(y12, 2) - 4 * x01 * Mathf.Pow(y12, 2) + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(y12, 2) +
			2 * Mathf.Pow(y01, 2) * Mathf.Pow(y12, 2) - 4 * y01 * y20 + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(y20, 2) + 2 * Mathf.Pow(y01, 2) * Mathf.Pow(y20, 2);
		mF[1, 2] = 2 * (1 - x01) * y01 + 2 * x01 * y01 + 2 * f5 * f7 + 2 * f5 * f1 +
			2 * f6 * f3 + 2 * f6 * f8;
		mF[1, 3] = 2 * (1 - x01) * x01 - 2 * Mathf.Pow(y01, 2) + 2 * f2 * f5 + 2 * f7 * f1 +
			2 * f6 * f4 + 2 * f3 * f8;
		mF[2, 0] = 2 * (1 - x01) * x01 - 2 * Mathf.Pow(y01, 2) + 2 * f2 * f5 + 2 * f7 * f1 +
			2 * f6 * f4 + 2 * f3 * f8;
		mF[2, 1] = 2 * (1 - x01) * y01 + 2 * x01 * y01 + 2 * f5 * f7 + 2 * f5 * f1 +
			2 * f6 * f3 + 2 * f6 * f8;
		mF[2, 2] = 2 + 4 * Mathf.Pow(x01, 2) - 4 * x12 + 4 * x01 * x12 + 2 * Mathf.Pow(x12, 2) - 4 * x01 * Mathf.Pow(x12, 2) + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(x12, 2) - 4 * Mathf.Pow(x01, 2) * x20 + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(x20, 2) +
			4 * Mathf.Pow(y01, 2) + 2 * Mathf.Pow(x12, 2) * Mathf.Pow(y01, 2) - 4 * x20 * Mathf.Pow(y01, 2) + 2 * Mathf.Pow(x20, 2) * Mathf.Pow(y01, 2) - 4 * y01 * y12 + 2 * Mathf.Pow(y12, 2) - 4 * x01 * Mathf.Pow(y12, 2) +
			2 * Mathf.Pow(x01, 2) * Mathf.Pow(y12, 2) + 2 * Mathf.Pow(y01, 2) * Mathf.Pow(y12, 2) + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(y20, 2) + 2 * Mathf.Pow(y01, 2) * Mathf.Pow(y20, 2);
		mF[2, 3] = 2 * f2 * f7 + 2 * f5 * f7 +
			2 * f6 * f8 + 2 * f4 * f8;
		mF[3, 0] = -2 * (1 - x01) * y01 - 2 * x01 * y01 + 2 * f2 * f7 + 2 * f2 * f1 +
			2 * f4 * f3 + 2 * f4 * f8;
		mF[3, 1] = 2 * (1 - x01) * x01 - 2 * Mathf.Pow(y01, 2) + 2 * f2 * f5 + 2 * f7 * f1 +
			2 * f6 * f4 + 2 * f3 * f8;
		mF[3, 2] = 2 * f2 * f7 + 2 * f5 * f7 +
			2 * f6 * f8 + 2 * f4 * f8;
		mF[3, 3] = 2 + 4 * Mathf.Pow(x01, 2) - 4 * x12 + 4 * x01 * x12 + 2 * Mathf.Pow(x12, 2) - 4 * x01 * Mathf.Pow(x12, 2) + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(x12, 2) - 4 * Mathf.Pow(x01, 2) * x20 + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(x20, 2) +
			4 * Mathf.Pow(y01, 2) + 2 * Mathf.Pow(x12, 2) * Mathf.Pow(y01, 2) - 4 * x20 * Mathf.Pow(y01, 2) + 2 * Mathf.Pow(x20, 2) * Mathf.Pow(y01, 2) - 4 * y01 * y12 + 2 * Mathf.Pow(y12, 2) - 4 * x01 * Mathf.Pow(y12, 2) +
			2 * Mathf.Pow(x01, 2) * Mathf.Pow(y12, 2) + 2 * Mathf.Pow(y01, 2) * Mathf.Pow(y12, 2) + 2 * Mathf.Pow(x01, 2) * Mathf.Pow(y20, 2) + 2 * Mathf.Pow(y01, 2) * Mathf.Pow(y20, 2);

		mF = mF.Invert();

		mC= Matrix.ZeroMatrix(4,6);
		mC[0, 0] = -2 * f1;
		mC[0, 1] = -2 * f2;
		mC[0, 2] = -2 * f3;
		mC[0, 3] = -2 * f4;
		mC[0, 4] = -2 * (1 - x01);
		mC[0, 5] = 2 * y01;
		mC[1, 0] = -2 * f5;
		mC[1, 1] = -2 * f1;
		mC[1, 2] = -2 * f6;
		mC[1, 3] = -2 * f3;
		mC[1, 4] = -2 * y01;
		mC[1, 5] = -2 * (1 - x01);
		mC[2, 0] = -2 * f7;
		mC[2, 1] = -2 * f5;
		mC[2, 2] = -2 * f8;
		mC[2, 3] = -2 * f6;
		mC[2, 4] = -2 * x01;
		mC[2, 5] = -2 * y01;
		mC[3, 0] = -2 * f2;
		mC[3, 1] = -2 * f7;
		mC[3, 2] = -2 * f4;
		mC[3, 3] = -2 * f8;
		mC[3, 4] = 2 * y01;
		mC[3, 5] = -2 * x01;
	}
}
