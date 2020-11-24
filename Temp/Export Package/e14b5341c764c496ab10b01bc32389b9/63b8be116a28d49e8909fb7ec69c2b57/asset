using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ARAPControl : MonoBehaviour
{
    [SerializeField]
    Button addPointButton;
    [SerializeField] Material lineMat;

    bool isAddpoint = true;

    Mesh themesh;
    int[] triangles;
    Vector3[] meshvertices;
    ARAPTriangle[] arapTris;
    int[] reorderVerID;//consister new order for each vertex uuuuuuqqqqqq

    List<int> mvConstraint;
    Vector3[] mvDeformed;

    Matrix scaleFreeMatrix;
    Matrix[] mHandmD;
    //deletebelow
    List<Vector3> drawTris;

    // Start is called before the first frame update
    void Start()
    {
        addPointButton.onClick.AddListener(SetAddPointState);

        themesh = GetComponent<MeshFilter>().sharedMesh;
        triangles = themesh.triangles;
        meshvertices = themesh.vertices;

        arapTris = new ARAPTriangle[triangles.Length/3];
        reorderVerID = new int[meshvertices.Length];
        mvConstraint = new List<int>();
        mvDeformed = meshvertices;

        ///precompute variables
        for (int i = 0; i < triangles.Length / 3; i++)
        {
            ARAPTriangle t = new ARAPTriangle();
            Vector3 v0 = meshvertices[triangles[i * 3]];
            Vector3 v1 = meshvertices[triangles[i * 3 + 1]];
            Vector3 v2 = meshvertices[triangles[i * 3 + 2]];

            var v2local = ARAPOperation.FindLocalCoordinator(v0, v1, v2);
            var v0local = ARAPOperation.FindLocalCoordinator(v1, v2, v0);
            var v1local = ARAPOperation.FindLocalCoordinator(v2, v0, v1);

            t.SetLocalXY(v0local, v1local, v2local);
            t.SetVertexID(triangles[i * 3], triangles[i * 3 + 1], triangles[i * 3 + 2]);

            t.scale_factor_divisor = Vector3.Dot(v1 - v0, v1 - v0);
            t.PreComputeScaleAdjustmentMatrixFC();

            arapTris[i] = t;
        }

        drawTris = new List<Vector3>();
        for (int i = 0; i < triangles.Length / 3; i++)
        {
            drawTris.Add(mvDeformed[triangles[i * 3]]);
            drawTris.Add(mvDeformed[triangles[i * 3 + 1]]);
            drawTris.Add(mvDeformed[triangles[i * 3 + 2]]);
        }

    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButton(0))
        {
            if (!isAddpoint)//move constraint point
            {
                int selected_id = FindHitVertex(Input.mousePosition);
                if (selected_id != -1 && mvConstraint.IndexOf(selected_id)!=-1)
                {
                    var newpos = Camera.main.ScreenToWorldPoint(new Vector3(Input.mousePosition.x, Input.mousePosition.y, 20));
                    mvDeformed[selected_id] = newpos;
                    MeshDeform();
                }

            }
            else //add constraint point
            {
                if (Input.GetMouseButtonDown(0))
                {
                    int selectedn = FindHitVertex(Input.mousePosition);
                    if (selectedn != -1)
                    {
                        if (mvConstraint.IndexOf(selectedn) == -1)
                        {
                            FindIsinConstraint(selectedn);
                            ReorderVertex();
                            scaleFreeMatrix = ARAPOperation.UpdateConstraint(meshvertices.Length, mvConstraint.Count, reorderVerID, arapTris);
                            mHandmD = ARAPOperation.PrecomputeFittingMatrix(meshvertices.Length, mvConstraint.Count, arapTris, reorderVerID);
                        }

                    }
                }
            }
        }
        themesh.vertices = mvDeformed;
    }

    void SetAddPointState()
    {
        isAddpoint = !isAddpoint;
        addPointButton.GetComponentInChildren<Text>().text = isAddpoint.ToString();
    }



    int FindHitVertex(Vector3 mpos)//find if the vertex is in mesh
    {
        int idx = -1;
        for(int i = 0; i < meshvertices.Length; i++)
        {
            Vector3 vPos = Camera.main.WorldToScreenPoint(meshvertices[i]);
            float dist = Vector2.Distance(vPos, mpos);
            if (dist < 30f) {  idx = i; }
        }
        return idx;
    }

    void FindIsinConstraint(int originalId)
    {
        if (mvConstraint.IndexOf(originalId) != -1) mvConstraint.Remove(originalId);
        else mvConstraint.Add(originalId);
    }

    void ReorderVertex()//call every time add constraint point
    {
        int nRow = 0;
        for (int i = 0; i < meshvertices.Length; i++)
        {
            if (mvConstraint.IndexOf(i) != -1)
            {
                continue;
            }
            reorderVerID[i] = nRow;
            nRow++;
        }
        if (nRow != meshvertices.Length - mvConstraint.Count) Debug.Log("wrong");
        for (int i = 0; i < mvConstraint.Count; i++)
        {
            int idx = mvConstraint[i];
            reorderVerID[idx] = nRow;
            nRow++;
        }
        if (nRow != meshvertices.Length) Debug.Log("wrong");
    }

    void MeshDeform()
    {
        int nConstaint = mvConstraint.Count;
        if (mvConstraint.Count > 1)
        {
            Matrix vq = new Matrix(2 * nConstaint, 1);
            for (int i = 0; i < nConstaint; i++)
            {
                vq[2 * i, 0] = mvDeformed[mvConstraint[i]].x;
                vq[2 * i + 1, 0] = mvDeformed[mvConstraint[i]].y;
            }
            //step1 deform
            var vu = scaleFreeMatrix * vq;
            int nVerts = meshvertices.Length;

            for (int i = 0; i < nVerts; i++)
            {
                if (mvConstraint.IndexOf(i) != -1)
                {
                    continue;
                }
                int nRow = reorderVerID[i];
                double fx = vu[2 * nRow, 0];
                double fy = vu[2 * nRow + 1, 0];
                mvDeformed[i] = new Vector3((float)fx, (float)fy, 0);
            }
            //step 2.1
            float[] f = new float[2 * meshvertices.Length];
            for (int i = 0; i < triangles.Length / 3; i++)
            {
                ARAPTriangle thetri = arapTris[i];
                Vector3[] updatedTris = ARAPOperation.AdjustScaleToTriangle(thetri, mvDeformed);

                for (int j = 0; j < 3; j++)
                {
                    var v0f = updatedTris[j];
                    var v1f = updatedTris[(j + 1) % 3];

                    int v0f_id = reorderVerID[thetri.vertex_id[j]];
                    int v1f_id = reorderVerID[thetri.vertex_id[(j + 1) % 3]];

                    f[2 * v0f_id] += -2 * v0f.x + 2 * v1f.x;
                    f[2 * v0f_id + 1] += -2 * v0f.y + 2 * v1f.y;
                    f[2 * v1f_id] += 2 * v0f.x - 2 * v1f.x;
                    f[2 * v1f_id + 1] += 2 * v0f.y - 2 * v1f.y;
                }
            }
            //step2.2
            int nFreeVerts = nVerts - nConstaint;
            Debug.Log(f[2 * nFreeVerts-1]);
            Debug.Log(f[2 * nFreeVerts]);
            Matrix f0 = Matrix.ZeroMatrix(2 * nFreeVerts, 1);
            for (int i = 0; i < 2 * nFreeVerts; i++)
            {
                f0[i, 0] = f[i];
            }
            for (int i = 0; i < nConstaint; i++)
            {
                vq[2 * i, 0] = mvDeformed[mvConstraint[i]].x;
                vq[2 * i + 1, 0] = mvDeformed[mvConstraint[i]].y;
            }
            Matrix mb = -1 * (mHandmD[1] * vq + f0);
            Matrix mu = mHandmD[0].SolveWith(mb);

            for (int i = 0; i < nVerts; i++)
            {
                if (mvConstraint.IndexOf(i) != -1)
                {
                    continue;
                }
                int nRow = reorderVerID[i];
                double fx = mu[2 * nRow, 0];
                double fy = mu[2 * nRow + 1, 0];
                mvDeformed[i] = new Vector3((float)fx, (float)fy, 0);
            }

            drawTris = new List<Vector3>();
            for ( int i = 0; i < triangles.Length / 3; i++)
            {
                drawTris.Add(mvDeformed[triangles[i * 3]]);
                drawTris.Add(mvDeformed[triangles[i * 3+1]]);
                drawTris.Add(mvDeformed[triangles[i * 3+2]]);
            }
        }
    }
    //delete below
    void OnRenderObject()
    {
        if (drawTris == null) return;

        GL.PushMatrix();
        GL.MultMatrix(transform.localToWorldMatrix);

        lineMat.SetColor("_Color", Color.black);
        lineMat.SetPass(0);
        GL.Begin(GL.LINES);
        for (int i = 0, n = drawTris.Count / 3; i < n; i++)
        {
            int j = i * 3;
            var a = drawTris[j]; var b = drawTris[j + 1]; var c = drawTris[j + 2];
            GL.Vertex(a); GL.Vertex(b);
            GL.Vertex(b); GL.Vertex(c);
            GL.Vertex(c); GL.Vertex(a);
        }
        GL.End();
        GL.PopMatrix();
    }


}
