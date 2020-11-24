using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ARAPOperation : MonoBehaviour
{

    public static Vector2 FindLocalCoordinator(Vector3 v0, Vector3 v1, Vector3 v2)
    {
        Vector3 v01 = v1 - v0;
        Vector3 v01R90 = Quaternion.AngleAxis(90, Vector3.forward) * v01;
        Vector3 vLocal = v2 - v0;
        float fx = Vector2.Dot(vLocal, v01) / (v01.magnitude * v01.magnitude);
        float fy = Vector2.Dot(vLocal, v01R90) / (v01R90.magnitude * v01R90.magnitude);
        Vector3 localCoordinator = new Vector2(fx, fy);
        Vector3 newv2 = v0 + localCoordinator.x * v01 + localCoordinator.y * v01R90;
        return localCoordinator;
    }

    //call every time add constraint point
    public static Matrix UpdateConstraint(int ninitialVer, int nContraint, int[]mVertexMap, ARAPTriangle[] triangles)
    {
        Matrix mFirstMatrix = Matrix.ZeroMatrix(1,1);
        //matrix computation
        int nfreeVec = ninitialVer - nContraint;
        if (nContraint > 1)
        {
            Matrix mG = Matrix.ZeroMatrix(2 * ninitialVer, 2 * ninitialVer);
            for (int i = 0; i < triangles.Length; i++)
            {
                ARAPTriangle tris = triangles[i];
                for (int j = 0; j < 3; j++)
                {
                    int v0_id = mVertexMap[tris.vertex_id[j]];
                    int v1_id = mVertexMap[tris.vertex_id[(j + 1) % 3]];
                    int v2_id = mVertexMap[tris.vertex_id[(j + 2) % 3]];

                    float x01 = tris.local_xy[(j+2)%3].x;
                    float y01 = tris.local_xy[(j + 2) % 3].y;

                    mG[2 * v0_id, 2 * v0_id] += Mathf.Pow(x01, 2) - 2 * x01 + Mathf.Pow(y01, 2) + 1;

                    mG[2 * v0_id, 2 * v0_id + 1] = 0 / 2;
                    mG[2 * v0_id + 1, 2 * v0_id] += 0;

                    mG[2 * v0_id, 2 * v1_id] += (1 - x01) * x01 - Mathf.Pow(y01, 2);
                    mG[2 * v1_id, 2 * v0_id] += (1 - x01) * x01 - Mathf.Pow(y01, 2);


                    mG[2 * v0_id, 2 * v1_id + 1] += -y01;
                    mG[2 * v1_id + 1, 2 * v0_id] += -y01;

                    mG[2 * v0_id, 2 * v2_id] += -(1 - x01);
                    mG[2 * v2_id, 2 * v0_id] += -(1 - x01);

                    mG[2 * v0_id, 2 * v2_id + 1] += y01;
                    mG[2 * v2_id + 1, 2 * v0_id] += y01;

                    mG[2 * v0_id + 1, 2 * v0_id + 1] += Mathf.Pow(x01, 2) - 2 * x01 + Mathf.Pow(y01, 2) + 1;

                    mG[2 * v0_id + 1, 2 * v1_id] += y01;
                    mG[2 * v1_id, 2 * v0_id + 1] += y01;

                    mG[2 * v0_id + 1, 2 * v1_id + 1] += x01 - x01 * x01 - y01 * y01;
                    mG[2 * v1_id + 1, 2 * v0_id + 1] += x01 - x01 * x01 - y01 * y01;

                    mG[2 * v0_id + 1, 2 * v2_id] += -y01;
                    mG[2 * v2_id, 2 * v0_id + 1] += -y01;

                    mG[2 * v0_id + 1, 2 * v2_id + 1] += -1 + x01;
                    mG[2 * v2_id + 1, 2 * v0_id + 1] += -1 + x01;

                    mG[2 * v1_id, 2 * v1_id] += Mathf.Pow(x01, 2) + Mathf.Pow(y01, 2);

                    mG[2 * v1_id, 2 * v1_id + 1] += 0 / 2;
                    mG[2 * v1_id + 1, 2 * v1_id] += 0 / 2;

                    mG[2 * v1_id, 2 * v2_id] += -x01;
                    mG[2 * v2_id, 2 * v1_id] += -x01;

                    mG[2 * v1_id, 2 * v2_id + 1] += -y01;
                    mG[2 * v2_id + 1, 2 * v1_id] += -y01;

                    mG[2 * v1_id + 1, 2 * v1_id + 1] += Mathf.Pow(x01, 2) + Mathf.Pow(y01, 2);

                    mG[2 * v1_id + 1, 2 * v2_id] += y01;
                    mG[2 * v2_id, 2 * v1_id + 1] += y01;

                    mG[2 * v1_id + 1, 2 * v2_id + 1] += -x01;
                    mG[2 * v2_id + 1, 2 * v1_id + 1] += -x01;


                    mG[2 * v2_id, 2 * v2_id] += 1;

                    mG[2 * v2_id, 2 * v2_id + 1] += 0;
                    mG[2 * v2_id + 1, 2 * v2_id] += 0;

                    mG[2 * v2_id + 1, 2 * v2_id + 1] += 1;
                }
            }
            Matrix mG00 = mG.SubMatrix(0, 0, 2 * nfreeVec, 2 * nfreeVec);
            Matrix mG01 = mG.SubMatrix(0, 2 * nfreeVec, 2 * nfreeVec, 2 * nContraint);
            Matrix mG10 = mG.SubMatrix(2 * nfreeVec, 0, 2 * nContraint, 2 * nfreeVec);

            Matrix mGprime = mG00 + Matrix.Transpose(mG00);
            Matrix mB = mG01 + Matrix.Transpose(mG10);

            Matrix mGprimeInverse = mGprime.Invert();

            mFirstMatrix = -1 * mGprimeInverse * mB;
        }
        return mFirstMatrix;
    }

    public static Vector3[] AdjustScaleToTriangle(ARAPTriangle thetriangle, Vector3[] mDrformedVertex)
    {
        Matrix triangle_points_coords = Matrix.ZeroMatrix(6, 1);
        for (int i = 0; i < 3; i++)
        {
            int vid = thetriangle.vertex_id[i];
            triangle_points_coords[i * 2, 0] = mDrformedVertex[vid].x;
            triangle_points_coords[i * 2 + 1, 0] = mDrformedVertex[vid].y;
        }

        Matrix theF = thetriangle.mF;
        Matrix theC = thetriangle.mC;

        Matrix vfittedcoords = -1 * theF * theC * triangle_points_coords;
        Vector3 v0f = new Vector3((float)vfittedcoords[0, 0], (float)vfittedcoords[1, 0], 0);
        Vector3 v1f = new Vector3((float)vfittedcoords[2, 0], (float)vfittedcoords[3, 0], 0);

        float x01 = thetriangle.local_xy[2].x;
        float y01 = thetriangle.local_xy[2].y;

        Vector3 v01 = v1f - v0f;
        Vector3 v01R90 = Quaternion.AngleAxis(90, Vector3.forward) * (v01);
        Vector3 v2f = v0f + x01 * (v01) + y01 * v01R90;

        Vector3[] updateTriPoints = new Vector3[3];
        updateTriPoints[0] = v0f; updateTriPoints[1] = v1f; updateTriPoints[2] = v2f;

        Vector3 centermass = (v0f + v1f + v2f) / 3;
        float sf = Mathf.Sqrt(thetriangle.scale_factor_divisor / Vector3.Dot(v01, v01));
        for (int i = 0; i < 3; i++)
        {
            updateTriPoints[i] -= centermass;
            updateTriPoints[i] *= sf;
            updateTriPoints[i] += centermass;
        }
        return updateTriPoints;

    }

    public static Matrix[] PrecomputeFittingMatrix(int nVerts, int nContraint, ARAPTriangle[] triangles, int[]mVertexMap)
    {
        int nFreevert = nVerts - nContraint;
        if (nContraint > 1 && nContraint < nVerts)
        {
            Matrix H = Matrix.ZeroMatrix(2 * nVerts, 2 * nVerts);
            for (int i = 0; i < triangles.Length; i++)
            {
                int[] tris = triangles[i].vertex_id;
                for (int j = 0; j < 3; j++)
                {
                    int v0p_id = mVertexMap[tris[j]];
                    int v1p_id = mVertexMap[tris[(j + 1) % 3]];

                    H[2 * v0p_id, 2 * v0p_id] += 1;
                    H[2 * v0p_id, 2 * v0p_id + 1] += 0;
                    H[2 * v0p_id, 2 * v1p_id] += -1;
                    H[2 * v0p_id, 2 * v1p_id + 1] += 0;
                    H[2 * v0p_id + 1, 2 * v0p_id] += 0;
                    H[2 * v0p_id + 1, 2 * v0p_id + 1] += 1;
                    H[2 * v0p_id + 1, 2 * v1p_id] += 0;
                    H[2 * v0p_id + 1, 2 * v1p_id + 1] += -1;
                    H[2 * v1p_id, 2 * v0p_id] += -1;
                    H[2 * v1p_id, 2 * v0p_id + 1] += 0;
                    H[2 * v1p_id, 2 * v1p_id] += 1;
                    H[2 * v1p_id, 2 * v1p_id + 1] += 0;
                    H[2 * v1p_id + 1, 2 * v0p_id] += 0;
                    H[2 * v1p_id + 1, 2 * v0p_id + 1] += -1;
                    H[2 * v1p_id + 1, 2 * v1p_id + 1] += 0;
                    H[2 * v1p_id + 1, 2 * v1p_id + 1] += 1;
                }
            }

            Matrix H00 = H.SubMatrix(0, 0, 2 * nFreevert, 2 * nFreevert);
            Matrix H01 = H.SubMatrix(0, 2 * nFreevert, 2 * nFreevert, 2 * nContraint);
            Matrix H10 = H.SubMatrix(2 * nFreevert, 0, 2 * nContraint, 2 * nFreevert);

            Matrix[] mHpandmD = new Matrix[2];

            mHpandmD[0] = H00 + Matrix.Transpose(H00);
            mHpandmD[1] = H01 + Matrix.Transpose(H10);

            return mHpandmD;
        }
        return null;

    }
}
