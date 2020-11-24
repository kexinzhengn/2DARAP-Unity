using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneBuilder : MonoBehaviour
{
    [SerializeField]
    private MeshFilter _meshFilter;

    [SerializeField]
    private MeshRenderer _meshRenderer;

    [SerializeField]
    private Vector2 _cellSize = new Vector2(1, 1);

    [SerializeField]
    private Vector2Int _gridSize = new Vector2Int(2, 2);

    public MeshRenderer MeshRenderer
    {
        get
        {
            return _meshRenderer;
        }
    }

    public MeshFilter MeshFilter
    {
        get
        {
            return _meshFilter;
        }
    }

    private void Awake()
    {
        _meshFilter = GetComponent<MeshFilter>();
        _meshRenderer = GetComponent<MeshRenderer>();
        UpdateMesh();
    }

    public void UpdateMesh()
    {
        Mesh mesh = new Mesh();

        Vector2 size;
        size.x = _cellSize.x * _gridSize.x;
        size.y = _cellSize.y * _gridSize.y;

        Vector2 halfSize = size / 2;

        List<Vector3> vertices = new List<Vector3>();
        List<Vector2> uvs = new List<Vector2>();

        Vector3 vertice = Vector3.zero;
        Vector2 uv = Vector3.zero;

        for (int y = 0; y < _gridSize.y + 1; y++)
        {
            vertice.y = y * _cellSize.y - halfSize.y;
            uv.y = y * _cellSize.y / size.y;

            for (int x = 0; x < _gridSize.x + 1; x++)
            {
                vertice.x = x * _cellSize.x - halfSize.x;
                uv.x = x * _cellSize.x / size.x;

                vertices.Add(vertice);
                uvs.Add(uv);
            }
        }

        int a = 0;
        int b = 0;
        int c = 0;
        int d = 0;
        int startIndex = 0;
        int[] indexs = new int[_gridSize.x * _gridSize.y * 2 * 3];
        for (int y = 0; y < _gridSize.y; y++)
        {
            for (int x = 0; x < _gridSize.x; x++)
            {

                a = y * (_gridSize.x + 1) + x;//0
                b = (y + 1) * (_gridSize.x + 1) + x;//1
                c = b + 1;//2
                d = a + 1;//3


                startIndex = y * _gridSize.x * 2 * 3 + x * 2 * 3;


                indexs[startIndex] = a;//0
                indexs[startIndex + 1] = b;//1
                indexs[startIndex + 2] = c;//2


                indexs[startIndex + 3] = c;//2
                indexs[startIndex + 4] = d;//3
                indexs[startIndex + 5] = a;//0
            }
        }

        //
        mesh.SetVertices(vertices);
        mesh.SetUVs(0, uvs);
        mesh.SetIndices(indexs, MeshTopology.Triangles, 0);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        mesh.RecalculateTangents();

        _meshFilter.mesh = mesh;
    }
}
