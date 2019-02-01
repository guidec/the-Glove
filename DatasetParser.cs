using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.IO.Ports;
using System;

public class DatasetParser : MonoBehaviour {
    [Header("Serial Connect")]
    //SerialPort sp = new SerialPort("COM5", 115200);     // Change Port
    SerialPort sp = new SerialPort("/dev/tty.usbmodem1421", 115200);     // Change Port
    public string rawInput = "";

    [Header("Sensitivity")]
    public float accelerometerSensitivity = 16384f;
    public float gyroscopeSensitivity = 131f;
    public float magnetometerSensitivity = 0.6f;
    public float truncateFactor = 1000;


    [Header("Debug Mode")]
    public bool debug;
    [SerializeField] float axDebug = 0f;
    [SerializeField] float ayDebug = 0f;
    [SerializeField] float azDebug = 0f;
    [SerializeField] float gxDebug = 0f;
    [SerializeField] float gyDebug = 0f;
    [SerializeField] float gzDebug = 0f;
    [SerializeField] float mxDebug = 0f;
    [SerializeField] float myDebug = 0f;
    [SerializeField] float mzDebug = 0f;

    // Parse parameters
    [Header("Parsing")]
    public TextAsset dataSet;
    private char lineSeparator = '\n';
    private char fieldSeparator = '\t';

    // Info
    private string[] rawStates;
    public int parseIndex = 3;
    string[] stateInfo;

    // IMU Dataset Consts
    [Header("IMU Processing")] 
    [SerializeField] int axIndex = 1;
    [SerializeField] int ayIndex = 2;
    [SerializeField] int azIndex = 3;
    [SerializeField] int gxIndex = 7;
    [SerializeField] int gyIndex = 8;
    [SerializeField] int gzIndex = 9;
    [SerializeField] int mxIndex = 10;
    [SerializeField] int myIndex = 11;
    [SerializeField] int mzIndex = 12;


    [SerializeField] float dt = 0f;
    [SerializeField] float ax = 0f;
    [SerializeField] float ay = 0f;
    [SerializeField] float az = 0f;
    [SerializeField] float gx = 0f;
    [SerializeField] float gy = 0f;
    [SerializeField] float gz = 0f;
    [SerializeField] float mx = 0f;
    [SerializeField] float my = 0f;
    [SerializeField] float mz = 0f;

    [Header("Tranformation")]
    Quaternion currRotation;
    public Transform cube;

    void Start () {
        Debug.Log(debug);
        if (!debug)
        {
            sp.Open();
            sp.ReadTimeout = 1;

        }
        ParseData();
        
        InvokeRepeating("HandleImu", 2f, 0.1f);
	}

	/*void FixedUpdate () {
        if (debug)
            DebugState();
        else
            ReadState();
        UpdateCube();
	}*/

    void HandleImu()
    {
        if (debug)
        {
            DebugState();
        }
        else
        {
            //ReadState();
            ReadFromConector();
        }
        UpdateCube();
    }

    void UpdateCube()
    {
        // Unity Compensation
        //currRotation = Cleaner.instance.GetQuaternionData(gz, gx, gy, az, ax, ay, mz, mx, my);

        // General
        currRotation = Cleaner.instance.GetQuaternionData(dt, gx, gy, gz, ax, ay, az, mx, my, mz);
        cube.rotation = currRotation;
    }

    void DebugState()
    {
        ax = axDebug/* / accelerometerSensitivity*/;
        ay = ayDebug /*/ accelerometerSensitivity*/;
        az = azDebug/* / accelerometerSensitivity*/;
        gx = gxDebug/* / gyroscopeSensitivity * Mathf.PI / 180*/;
        gy = gyDebug/* / gyroscopeSensitivity * Mathf.PI / 180*/;
        gz = gzDebug /*/ gyroscopeSensitivity * Mathf.PI / 180*/;
        mx = mxDebug /*/ magnetometerSensitivity*/;
        my = myDebug /*/ magnetometerSensitivity*/;
        mz = mzDebug /*/ magnetometerSensitivity*/;
    }
/*
public IEnumerator AsynchronousReadFromArduino(Action<string> callback, Action fail = null, float timeout = float.PositiveInfinity) {
    DateTime initialTime = DateTime.Now;
    DateTime nowTime;
    TimeSpan diff = default(TimeSpan);

    string dataString = null;

    do {
        try {
            dataString = sp.ReadLine();
        }
        catch (TimeoutException) {
            dataString = null;
        }

        if (dataString != null)
        {
            callback(dataString);
            yield break; // Terminates the Coroutine
        } else
            yield return null; // Wait for next frame

        nowTime = DateTime.Now;
        diff = nowTime - initialTime;

    } while (diff.Milliseconds < timeout);

    if (fail != null)
        fail();
    yield return null;
}*/

    void ReadFromConector()
    {
        
        rawInput = sp.ReadLine();
        /*
        StartCoroutine(AsynchronousReadFromArduino(
        (string s) => rawInput=s,     // Callback
        () => Debug.LogError("Error!"), // Error callback
        10000f                          // Timeout (milliseconds)
        ));
        */
        //print(rawInput);
        stateInfo = rawInput.Split('\t');
        //print((float.Parse(stateInfo[4]) /*/ gyroscopeSensitivity*/) + " " 
        //    + (float.Parse(stateInfo[5]) /*/ gyroscopeSensitivity*/) + " " 
        //    + (float.Parse(stateInfo[6]) /*/ gyroscopeSensitivity*/));
        dt = float.Parse(stateInfo[0]);
        ax = float.Parse(stateInfo[1]) / accelerometerSensitivity;
        ay = float.Parse(stateInfo[2]) / accelerometerSensitivity;
        az = float.Parse(stateInfo[3]) / accelerometerSensitivity;
        //gx = float.Parse(stateInfo[4]) / gyroscopeSensitivity * Mathf.PI/180;
        //gy = float.Parse(stateInfo[5]) / gyroscopeSensitivity * Mathf.PI / 180;
        //gz = float.Parse(stateInfo[6]) / gyroscopeSensitivity * Mathf.PI / 180; 
        //print(""+stateInfo[4]+" "+stateInfo[5]+" "+stateInfo[6]);
        gz = float.Parse(stateInfo[4]);
        gy = float.Parse(stateInfo[5]);
        gx = float.Parse(stateInfo[6]);
        
        mx = float.Parse(stateInfo[7]) / magnetometerSensitivity;
        my = float.Parse(stateInfo[8]) / magnetometerSensitivity;
        mz = float.Parse(stateInfo[9]) / magnetometerSensitivity;

        // Truncate
        ax = Mathf.Round(ax * truncateFactor) / truncateFactor;
        ay = Mathf.Round(ay * truncateFactor) / truncateFactor;
        az = Mathf.Round(az * truncateFactor) / truncateFactor;
        //gx = Mathf.Round(gx * truncateFactor) / truncateFactor;
        //gy = Mathf.Round(gy * truncateFactor) / truncateFactor;
        //gz = Mathf.Round(gz * truncateFactor) / truncateFactor;
        mx = Mathf.Round(mx * truncateFactor) / truncateFactor;
        my = Mathf.Round(my * truncateFactor) / truncateFactor;
        mz = Mathf.Round(mz * truncateFactor) / truncateFactor;
        
    }

    void ReadState()
    {
        if (parseIndex >= rawStates.Length) return;
        stateInfo = rawStates[parseIndex].Split(fieldSeparator);
        //print(stateInfo[1]);
        ax = float.Parse(stateInfo[axIndex]) / accelerometerSensitivity;
        ay = float.Parse(stateInfo[ayIndex]) / accelerometerSensitivity;
        az = float.Parse(stateInfo[azIndex]) / accelerometerSensitivity;
        //gx = float.Parse(stateInfo[gxIndex]) ;// gyroscopeSensitivity * Mathf.PI / 180;
        //gy = float.Parse(stateInfo[gyIndex]) ;// gyroscopeSensitivity * Mathf.PI / 180;
        //gz = float.Parse(stateInfo[gzIndex]) ;// gyroscopeSensitivity * Mathf.PI / 180;
        gx=float.Parse(rawStates[gxIndex]);
        gy=float.Parse(rawStates[gyIndex]);
        gz=float.Parse(rawStates[gzIndex]);
        mx = float.Parse(stateInfo[mxIndex]) / magnetometerSensitivity;
        my = float.Parse(stateInfo[myIndex]) / magnetometerSensitivity;
        mz = float.Parse(stateInfo[mzIndex]) / magnetometerSensitivity;
        //print(ax + " " + ay + " " + az + " " + gx + " " + gy + " " + gz + " " + mx + " " + my + " " + mz);

        // Truncate
        ax = Mathf.Round(ax * truncateFactor) / truncateFactor;
        ay = Mathf.Round(ay * truncateFactor) / truncateFactor;
        az = Mathf.Round(az * truncateFactor) / truncateFactor;
        //gx = Mathf.Round(gx * truncateFactor) / truncateFactor;
        //gy = Mathf.Round(gy * truncateFactor) / truncateFactor;
        //gz = Mathf.Round(gz * truncateFactor) / truncateFactor;
        mx = Mathf.Round(mx * truncateFactor) / truncateFactor;
        my = Mathf.Round(my * truncateFactor) / truncateFactor;
        mz = Mathf.Round(mz * truncateFactor) / truncateFactor;

        parseIndex++;
    }

    void ParseData()
    {
        rawStates = dataSet.text.Split(lineSeparator);
    }
}
