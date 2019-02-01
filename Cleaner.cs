using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.IO.Ports;

public class Cleaner : MonoBehaviour {
    //SerialPort ARM = new SerialPort("COM5", 115200);     // Change Port
    SerialPort ARM = new SerialPort("/dev/tty.usbmodem1411", 115200);     // Change Port
    private static Cleaner inst;
    public static Cleaner instance { get { return inst; } }

    private void Awake()
    {
        if (inst != null && inst != this)
            Destroy(this.gameObject);
        else
            inst = this;
    }

    /// <summary>
    /// Gets or sets the sample period.
    /// </summary>
    [SerializeField] public float SamplePeriod = 512;

    /// <summary>
    /// State, x, y, z, d/dx, d/dy, d/dz
    /// </summary>
    [SerializeField] public float[] state = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    [SerializeField] public float[] calibrateValues = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    

    /// <summary>
    /// Motor state x,y,z
    /// </summary>
    [SerializeField] public int[] motorState = {0,0,0};

    /// <summary>
    /// Gets or sets the algorithm gain beta.
    /// </summary>
    [SerializeField] public float Beta = 0.1f;

    /// <summary>
    /// Gets or sets the Quaternion output.
    /// </summary>
    [SerializeField] public float[] quat;
    [SerializeField] public float beginTime = 0.0f;
    public bool transition= false;
    void Start () {
        ARM.Open();
    }

    public Quaternion GetQuaternionData(float dt, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
        //Debug.Log("GetQuaternionData "+ gx);
        /*
        if(beginTime<5.0f){
            calibrate(dt, gx, gy, gz, ax, ay, az, mx, my, mz);
        } else {
            if(transition == false){
                transition = true;
                state[3]= -calibrateValues[3];
                state[4]= -calibrateValues[4];
                state[5]= -calibrateValues[5];
                calibrateValues[0]= -calibrateValues[0]/dt;
                calibrateValues[2]= -calibrateValues[1]/dt;
                calibrateValues[1]= -calibrateValues[2]/dt;
            }
            UpdateImu(dt, gx, gy, gz, ax, ay, az, mx, my, mz);
        }*/
        UpdateImu(dt, gx, gy, gz, ax, ay, az, mx, my, mz);

        Quaternion retQuat = new Quaternion(quat[0], quat[1], quat[2], quat[3]);
        return retQuat;
    }

    /// <summary>
    /// Algorithm AHRS update method. Requires only gyroscope and accelerometer data.
    /// </summary>
    /// <param name="gx">
    /// Gyroscope x axis measurement in radians/s.
    /// </param>
    /// <param name="gy">
    /// Gyroscope y axis measurement in radians/s.
    /// </param>
    /// <param name="gz">
    /// Gyroscope z axis measurement in radians/s.
    /// </param>
    /// <param name="ax">
    /// Accelerometer x axis measurement in any calibrated units.
    /// </param>
    /// <param name="ay">
    /// Accelerometer y axis measurement in any calibrated units.
    /// </param>
    /// <param name="az">
    /// Accelerometer z axis measurement in any calibrated units.
    /// </param>
    /// <param name="mx">
    /// Magnetometer x axis measurement in any calibrated units.
    /// </param>
    /// <param name="my">
    /// Magnetometer y axis measurement in any calibrated units.
    /// </param>
    /// <param name="mz">
    /// Magnetometer z axis measurement in any calibrated units.
    /// </param>
    /// <remarks>
    /// Optimised for minimal arithmetic.
    /// Total ±: 160
    /// Total *: 172
    /// Total /: 5
    /// Total sqrt: 5
    /// </remarks> 
    public void calibrate(float dt, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
        float h = 0.0017f;
        float g = 0.0040f;
        if(beginTime > 0.01f){
            float place = dt;
            dt = dt-beginTime;
            beginTime = place;
        } 
        //float dt = 0.1048f;
        //gx -= 25.048
        //gy += 60.47
        //gz -= 2.4
        //gx-=2.096f;
        //gy+=59.48f;
        //gz-=25.84f;
        calibrateValues[0]+=gx;
        calibrateValues[1]+=gy;
        calibrateValues[2]+=gz;
        calibrateValues[6]+=dt;

        //print(""+gx+" "+gy+" "+gz);
        print(beginTime);
        float predictionx = calibrateValues[0]+dt*calibrateValues[3];
        float predictiony = calibrateValues[1]+dt*calibrateValues[4];
        float predictionz = calibrateValues[2]+dt*calibrateValues[5];

        float residualx = gx - calibrateValues[3];
        float residualy = gy - calibrateValues[4];
        float residualz = gz - calibrateValues[5];

        calibrateValues[3] = h*(residualx*dt);
        calibrateValues[4] = h*(residualy*dt);
        calibrateValues[5] = h*(residualz*dt);
        float prevx=calibrateValues[0];
        float prevy=calibrateValues[1];
        float prevz=calibrateValues[2];
        calibrateValues[0] = calibrateValues[0]+g*residualx;
        calibrateValues[1] = calibrateValues[1]+g*residualy;
        calibrateValues[2] = calibrateValues[2]+g*residualz;
    }
    public void UpdateImu(float dt, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
    {
        
        float h = 0.0017f;
        float g = 0.0030f;
        if(beginTime > 0.01f){
            float place = dt;
            dt = dt-beginTime;
            beginTime = place;
        } else {
            beginTime+=dt;
        }
        //float dt = 0.1048f;
        //gx -= 25.048
        //gy += 60.47
        //gz -= 2.4
        gx-=2.096f;
        gy+=59.48f;
        gz-=25.84f;
        print(""+gx+" "+gy+" "+gz);

        float predictionx = state[0]+dt*state[3];
        float predictiony = state[1]+dt*state[4];
        float predictionz = state[2]+dt*state[5];

        float residualx = gx - state[3];
        float residualy = gy - state[4];
        float residualz = gz - state[5];

        state[3] = h*(residualx*dt);
        state[4] = h*(residualy*dt);
        state[5] = h*(residualz*dt);
        float prevx=state[0];
        float prevy=state[1];
        float prevz=state[2];
        state[0] = state[0]+g*residualx;
        state[1] = state[1]+g*residualy;
        state[2] = state[2]+g*residualz;
        Debug.Log(""+state[0]+" "+state[1]+" "+state[2]+" "+state[3]+" "+state[4]+" "+state[5]);
        Quaternion rotation = Quaternion.Euler(state[2], state[0], state[1]);
        quat[0] = rotation.w;
        quat[1] = rotation.x;
        quat[2] = rotation.y;
        quat[3] = rotation.z;



        prevx=Mathf.Round(state[0]/1.8f-motorState[0]);
        prevy=Mathf.Round(state[1]/1.8f-motorState[1]);
        prevz=Mathf.Round(state[2]/1.8f-motorState[2]);
        motorState[0]+=(int)prevx;
        motorState[1]+=(int)prevy;
        motorState[2]+=(int)prevz;
        //print(prevx);
        int roll = (int)Mathf.Round(prevx);
        int pitch = (int)Mathf.Round(prevy);
        int yaw = (int)Mathf.Round(prevz);
        
        //byte roll = (byte)Mathf.Round((((Mathf.Round(prevx/1.8f))/200.0f)-(int)(Mathf.Round(prevx/1.8f)/200.0f))*200.0f);
        //byte pitch= (byte)Mathf.Round((((Mathf.Round(prevy/1.8f))/200.0f)-(int)(Mathf.Round(prevy/1.8f)/200.0f))*200.0f);
        //byte yaw= (byte)Mathf.Round((((Mathf.Round(prevz/1.8f))/200.0f)-(int)(Mathf.Round(prevz/1.8f)/200.0f))*200.0f);

        if((int)roll>100)roll = (300-roll);
        else if((int)roll>=0)roll = roll;
        else if((int)roll>=-100)roll=(Mathf.Abs(roll)+100);
        else if((int)roll>=-200)roll=(200-Mathf.Abs(roll));

        if((int)pitch>100)pitch =(300-pitch);
        else if((int)pitch>=0)pitch = pitch;
        else if((int)pitch>=-100)pitch=(Mathf.Abs(pitch)+100);
        else if((int)pitch>=-200)pitch=(200-Mathf.Abs(pitch));
        if((int)yaw>100)yaw = (300-yaw);
        else if((int)yaw>=0)yaw = yaw;
        else if((int)yaw>=-100)yaw=(Mathf.Abs(yaw)+100);
        else if((int)yaw>=-200)yaw=(200-Mathf.Abs(yaw));

        motorState[0] = (int)Mathf.Round(prevx);
        motorState[1] = (int)Mathf.Round(prevy);
        motorState[2] = (int)Mathf.Round(prevz);

        /*
        if(Mathf.Round(prevx/1.8f)>=0 && Mathf.Round(prevx/1.8f)<=100){ //rounded to nearest step -> greater than or equal to 0 steps or less than or equal to 100
             roll = (byte)Mathf.Round(prevx/1.8f);
        } else if(Mathf.Round(prevx/1.8f)<0 && Mathf.Round(prevx/1.8f)>100){
             roll = (byte)(Mathf.Round(prevx/1.8f)-200);
        } else if(Mathf.Round(prevx/1.8f)<-100){
             roll = (byte)Mathf.Round(prevx/1.8f);
        } else {
            Debug.Log("Cleaner Line 131 DebugX-Roll");
            
        }
        if(Mathf.Round(prevy/1.8f)>=0 && Mathf.Round(prevy/1.8f)<=100){ //rounded to nearest step -> greater than or equal to 0 steps or less than or equal to 100
             pitch = (byte)Mathf.Round(prevy/1.8f);
        } else if(Mathf.Round(prevx/1.8f)<0 && Mathf.Round(prevy/1.8f)>100){
             pitch = (byte)(Mathf.Round(prevy/1.8f)-200);
        } else if(Mathf.Round(prevy /1.8f)<-100){
             pitch = (byte)Mathf.Round(prevy/1.8f);
        } else {
            Debug.Log("Cleaner Line 131 DebugY-Pitch");
            
        }
        if(Mathf.Round(prevz/1.8f)>=0 && Mathf.Round(prevz/1.8f)<=100){ //rounded to nearest step -> greater than or equal to 0 steps or less than or equal to 100
             yaw = (byte)Mathf.Round(prevz/1.8f);
        } else if(Mathf.Round(prevz/1.8f)<0 && Mathf.Round(prevz/1.8f)>100){
             yaw = (byte)(Mathf.Round(prevz/1.8f)-200);
        } else if(Mathf.Round(prevz/1.8f)<-100){
             yaw = (byte)Mathf.Round(prevz/1.8f);
        } else {
            Debug.Log("Cleaner Line 131 DebugZ-Yaw");
            
        }*/

        byte[] array = {(byte)roll,(byte)pitch,(byte)yaw};
        //Debug.Log(roll+", "+pitch+", "+yaw);
        //Debug.Log(motorState[0]+", "+motorState[1]+", "+motorState[2]);
        WriteToArduino(array,0,3);
        

        /*
        float q1 = quat[0], q2 = quat[1], q3 = quat[2], q4 = quat[3];   // short name local variable for readability
        float norm;
        float hx, hy, _2bx, _2bz;
        float s1, s2, s3, s4;
        float qDot1, qDot2, qDot3, qDot4;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q1mx;
        float _2q1my;
        float _2q1mz;
        float _2q2mx;
        float _4bx;
        float _4bz;
        float _2q1 = 2f * q1;
        float _2q2 = 2f * q2;
        float _2q3 = 2f * q3;
        float _2q4 = 2f * q4;
        float _2q1q3 = 2f * q1 * q3;
        float _2q3q4 = 2f * q3 * q4;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q1q4 = q1 * q4;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q2q4 = q2 * q4;
        float q3q3 = q3 * q3;
        float q3q4 = q3 * q4;
        float q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = (float)Mathf.Sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0f) return; // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = (float)Mathf.Sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0f) return; // handle NaN
        norm = 1 / norm;        // use reciprocal for division
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        _2q1mx = 2f * q1 * mx;
        _2q1my = 2f * q1 * my;
        _2q1mz = 2f * q1 * mz;
        _2q2mx = 2f * q2 * mx;
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
        _2bx = (float)Mathf.Sqrt(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
        _4bx = 2f * _2bx;
        _4bz = 2f * _2bz;

        // Gradient decent algorithm corrective step
        s1 = -_2q3 * (2f * q2q4 - _2q1q3 - ax) + _2q2 * (2f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2f * q2q4 - _2q1q3 - ax) + _2q1 * (2f * q1q2 + _2q3q4 - ay) - 4f * q2 * (1 - 2f * q2q2 - 2f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2f * q2q4 - _2q1q3 - ax) + _2q4 * (2f * q1q2 + _2q3q4 - ay) - 4f * q3 * (1 - 2f * q2q2 - 2f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2f * q2q4 - _2q1q3 - ax) + _2q3 * (2f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        norm = 1f / (float)Mathf.Sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - Beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - Beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - Beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - Beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * SamplePeriod;
        q2 += qDot2 * SamplePeriod;
        q3 += qDot3 * SamplePeriod;
        q4 += qDot4 * SamplePeriod;
        norm = 1f / (float)Mathf.Sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        quat[0] = q1 * norm;
        quat[1] = q2 * norm;
        quat[2] = q3 * norm;
        quat[3] = q4 * norm;
        */
    }
    

    public void WriteToArduino(byte[] message, int offset, int count) {
        ARM.Write(message,offset,count);
        ARM.BaseStream.Flush();
        
    }
}
