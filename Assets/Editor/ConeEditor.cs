using UnityEngine;
using UnityEditor;
using System.Collections;

[CustomEditor(typeof(Cone))]
public class ConeEditor : Editor
{
	[MenuItem("GameObject/3D Object/Cone")]
	static void Create() {
		GameObject gameObject = new GameObject("Cone");
		Cone cone = gameObject.AddComponent<Cone>();
		gameObject.AddComponent<MeshCollider>();
		gameObject.AddComponent<MeshFilter>();
		MeshRenderer renderer = gameObject.AddComponent<MeshRenderer>();
		renderer.sharedMaterial = new Material(Shader.Find("Standard"));
		renderer.sharedMaterial.SetColor("_Color", Color.white);
		// build mesh
		cone.BuildMesh();
	}

	public override void OnInspectorGUI() {
		base.DrawDefaultInspector();

		Cone cone = target as Cone;
		if (cone == null) {
			return;
		}

		EditorGUILayout.BeginHorizontal();

		// Rebuild mesh when user click the Rebuild button
		if (GUILayout.Button("Rebuild")) {
			// check parameter
			if (cone.CheckParam()) {
				// update parameter
				cone.UpdateParam(false);
				// build mesh
				cone.BuildMesh(); 
			}
		}

		// Reset to last successul build parameter
		if (GUILayout.Button("Reset")) {
			cone.UpdateParam(true);
		}

		EditorGUILayout.EndHorizontal();
	}
}

public class Cone : MonoBehaviour
{
	public float height = 2.0f;
	public float top_radius = 0.0f;
	public float bottom_radius = 1.0f;
	public int sections = 30;

	private float _height = 2.0f;
	private float _top_radius = 0.0f;
	private float _bottom_radius = 1.0f;
	private int _sections = 30;

	
	public bool CheckParam() {
		// check some condition
		if (sections < 3) {
			Debug.LogError("sections must be 3 or more!");
			return false;
		}
		if (top_radius < 0.0f) {
			Debug.LogWarning("top_radius is negative, get absolute value...");
			top_radius = Mathf.Abs(top_radius);
		}
		if (bottom_radius < 0.0f) {
			Debug.LogWarning("bottom_radius is negative, get absolute value...");
			bottom_radius = Mathf.Abs(bottom_radius);
		}
		if (top_radius == bottom_radius) {
			Debug.LogError("top_radius and bottom_radius are the same, please use a Cylinder!");
			return false;
		}
		if (top_radius > bottom_radius) {
			Debug.Log("top_radius is bigger than bottom_radius, exchange value...");
			float tmp = top_radius;
			top_radius = bottom_radius;
			bottom_radius = tmp;
		}
		if (height == _height && top_radius == _top_radius &&
		    bottom_radius == _bottom_radius && sections == _sections) {
			Debug.Log("Nothing changed...");
			return false;
		}
		return true;
	}

	public void UpdateParam(bool isReset) {
		// update parameter
		if (isReset) {
			height = _height;
			top_radius = _top_radius;
			bottom_radius = _bottom_radius;
			sections = _sections;
		} else {
			_height = height;
			_top_radius = top_radius;
			_bottom_radius = bottom_radius;
			_sections = sections;
		}
	}

	public void BuildMesh() {
		// Get meshFilter
		MeshFilter meshFilter = this.GetComponent<MeshFilter>();
		if (meshFilter == null) {
			Debug.LogError("MeshFilter not found!");
			return;
		}
		// Get / Create a empty Mesh
		Mesh mesh = meshFilter.sharedMesh;
		if (mesh == null) {
			meshFilter.mesh = new Mesh();
			mesh = meshFilter.sharedMesh;
		}
		mesh.Clear();

		// define some value
		Vector3[] vertices;
		int[] triangles;

		if (_top_radius == 0.0f) {
			// it's a full circular cone

			// start calculate verticles
			vertices = new Vector3[_sections + 2]; // a circle, circle center and an apex

			// circle
			float step = 2 * Mathf.PI / _sections;
			float angle = 0;

			for (int i = 0; i < _sections; i++) {
				float circle_x = Mathf.Sin(angle) * _bottom_radius;
				float circle_z = Mathf.Cos(angle) * _bottom_radius;
				angle += step; // increase angle

				vertices[i] = new Vector3(circle_x, 0, circle_z);
			}
			vertices[_sections] = new Vector3(0, _height, 0); // apex
			vertices[_sections + 1] = new Vector3(0, 0, 0); // circle center

			// start calculate triangles
			triangles = new int[_sections * 2 * 3];

			int index_triangles = 0;

			for (int i = 1; i < _sections; i++) {
				// side faces
				triangles[index_triangles] = _sections; // apex
				triangles[index_triangles + 1] = i - 1;
				triangles[index_triangles + 2] = i;
				index_triangles += 3;
				// base face
				triangles[index_triangles] = _sections + 1; // circle center
				triangles[index_triangles + 1] = i;
				triangles[index_triangles + 2] = i - 1;
				index_triangles += 3;
			}
			// side faces
			triangles[index_triangles] = _sections; // apex
			triangles[index_triangles + 1] = _sections - 1;
			triangles[index_triangles + 2] = 0;
			index_triangles += 3;
			// base face
			triangles[index_triangles] = _sections + 1; // circle center
			triangles[index_triangles + 1] = 0;
			triangles[index_triangles + 2] = _sections - 1;
			//index_triangles += 3;

		} else {
			// it's a truncated circular cone

			// start calculate verticles
			vertices = new Vector3[_sections * 2 + 2]; // two circles and two circle center point

			// circle
			float step = 2 * Mathf.PI / _sections;
			float top_angle = 0;
			float bottom_angle = step / 2;

			for (int i = 0; i < _sections * 2; i += 2) {
				float top_circle_x = Mathf.Sin(top_angle) * _top_radius;
				float top_circle_z = Mathf.Cos(top_angle) * _top_radius;
				float bottom_circle_x = Mathf.Sin(bottom_angle) * _bottom_radius;
				float bottom_circle_z = Mathf.Cos(bottom_angle) * _bottom_radius;
				top_angle += step; // increase angle
				bottom_angle += step;

				vertices[i] = new Vector3(top_circle_x, _height, top_circle_z);
				vertices[i + 1] = new Vector3(bottom_circle_x, 0, bottom_circle_z);
			}
			vertices[_sections * 2] = new Vector3(0, _height, 0); // top circle center
			vertices[_sections * 2 + 1] = new Vector3(0, 0, 0); // bottom circle center

			// start calculate triangles
			triangles = new int[_sections * 4 * 3];

			int index_triangles = 0;

			for (int i = 2; i < _sections * 2; i += 2) {
				// side faces
				triangles[index_triangles] = i - 2;
				triangles[index_triangles + 1] = i - 1;
				triangles[index_triangles + 2] = i;
				index_triangles += 3;
				triangles[index_triangles] = i;
				triangles[index_triangles + 1] = i - 1;
				triangles[index_triangles + 2] = i + 1;
				index_triangles += 3;
				// base face
				triangles[index_triangles] = _sections * 2; // top circle center
				triangles[index_triangles + 1] = i - 2;
				triangles[index_triangles + 2] = i;
				index_triangles += 3;
				triangles[index_triangles] = _sections * 2 + 1; // bottom circle center
				triangles[index_triangles + 1] = i + 1;
				triangles[index_triangles + 2] = i - 1;
				index_triangles += 3;
			}
			// side faces
			triangles[index_triangles] = _sections * 2 - 2;
			triangles[index_triangles + 1] = _sections * 2 - 1;
			triangles[index_triangles + 2] = 0;
			index_triangles += 3;
			triangles[index_triangles] = 0;
			triangles[index_triangles + 1] = _sections * 2 - 1;
			triangles[index_triangles + 2] = 1;
			index_triangles += 3;
			// base face
			triangles[index_triangles] = _sections * 2; // top circle center
			triangles[index_triangles + 1] = _sections * 2 - 2;
			triangles[index_triangles + 2] = 0;
			index_triangles += 3;
			triangles[index_triangles] = _sections * 2 + 1; // bottom circle center
			triangles[index_triangles + 1] = 1;
			triangles[index_triangles + 2] = _sections * 2 - 1;
			//index_triangles += 3;
		}

		// set mesh
		mesh.vertices = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals();
		mesh.RecalculateBounds();
	}
}