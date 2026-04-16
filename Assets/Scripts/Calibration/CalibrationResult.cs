// Author: Jackson Russell

using System;
using System.IO;
using UnityEngine;

/// Stores the result of a completed hand-eye calibration session and provides
/// JSON serialisation to Application.persistentDataPath.
[Serializable]
public class CalibrationResult
{
    // Path within Application.persistentDataPath where the result is saved.
    public const string FileName = "hand_eye_calibration.json";

    // ISO 8601 timestamp of when the calibration was completed.
    public string timestamp;

    // Number of motion pairs used in the solve.
    public int pairsUsed;

    // Mean rotation residual across all pairs in degrees. Values below 2 deg
    // indicate a well-conditioned result; values above 5 deg suggest poor
    // waypoint diversity or unstable tag detection.
    public float residualDeg;

    // T_tool0_to_camera stored as 16 column-major floats (Unity Matrix4x4 layout).
    public float[] matrixValues = new float[16];

    // Returns the calibration as a Matrix4x4. The matrix is only valid after Load or Solve.
    public Matrix4x4 ToMatrix4x4()
    {
        Matrix4x4 m = new Matrix4x4();
        for (int i = 0; i < 16; i++)
            m[i] = matrixValues[i];
        return m;
    }

    // Fills matrixValues from a Matrix4x4.
    public void FromMatrix4x4(Matrix4x4 m)
    {
        for (int i = 0; i < 16; i++)
            matrixValues[i] = m[i];
    }

    // Returns the translation component of the stored transform.
    public Vector3 Translation => new Vector3(matrixValues[12], matrixValues[13], matrixValues[14]);

    // Returns the rotation component of the stored transform.
    public Quaternion Rotation => ToMatrix4x4().rotation;

    // Saves this result to Application.persistentDataPath as a JSON file.
    // Returns the full path written on success, or null on failure.
    public string Save()
    {
        try
        {
            string path = Path.Combine(Application.persistentDataPath, FileName);
            string json = JsonUtility.ToJson(this, prettyPrint: true);
            File.WriteAllText(path, json);
            Debug.Log("[CalibrationResult] Saved to: " + path);
            return path;
        }
        catch (Exception ex)
        {
            Debug.LogError("[CalibrationResult] Save failed: " + ex.Message);
            return null;
        }
    }

    // Loads the most recent calibration from Application.persistentDataPath.
    // Returns null if no file exists or the file is malformed.
    public static CalibrationResult Load()
    {
        string path = Path.Combine(Application.persistentDataPath, FileName);
        if (!File.Exists(path))
        {
            Debug.Log("[CalibrationResult] No saved calibration found at: " + path);
            return null;
        }

        try
        {
            string         json   = File.ReadAllText(path);
            CalibrationResult result = JsonUtility.FromJson<CalibrationResult>(json);
            Debug.Log("[CalibrationResult] Loaded from: " + path
                + "  residual=" + result.residualDeg.ToString("F2") + " deg"
                + "  pairs=" + result.pairsUsed
                + "  time=" + result.timestamp);
            return result;
        }
        catch (Exception ex)
        {
            Debug.LogError("[CalibrationResult] Load failed: " + ex.Message);
            return null;
        }
    }

    // Returns true if a saved calibration file exists on disk.
    public static bool Exists()
        => File.Exists(Path.Combine(Application.persistentDataPath, FileName));
}
