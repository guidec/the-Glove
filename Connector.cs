/* 
 * Copyright 2018, Raahul Natarrajan, All Rights Reserved.
 * Z Forward/Backward, Y Up/Down, X Right/Left 
 * Unity Axes for rotation
 * Send Arduino Input for each hand separated by brackets
 * Format:
 * <handL></handL>
 */


using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;
using System;

public class Connector : MonoBehaviour {
    public bool debugInput;
    SerialPort sp = new SerialPort("COM3", 9600);
    //SerialPort sp = new SerialPort("/dev/cu.usbmodem1421",115200);
    public string rawInput = "";
    [SerializeField] public string[] separators;
    public Hand leftHand;
    public Hand rightHand;

    public enum JointType
    {
        leftHand, rightHand, thumb, index, middle, ring, pinky, wrist
    }

    void Start()
    {
        if (!debugInput)
        {
            sp.Open();
            sp.ReadTimeout = 1;
        }
    }

    void Update()
    {
        if (!debugInput)
        {
            try
            {
                print(sp.ReadLine());
            }
            catch (System.Exception)
            {
            }
        }

        ParseInput();
    }

    void ParseInput()
    {
        GetHandData(leftHand, rawInput);
        GetHandData(rightHand, rawInput);
        leftHand.UpdateHand();
        rightHand.UpdateHand();
    }

    void GetHandData(Hand h, string rawData)
    {
        JointType j = h.jointType;
        string startTag = GetStartTag(j);
        string endTag = GetEndTag(j);
        int start = rawData.IndexOf(startTag) + startTag.Length;
        int end = rawData.IndexOf(endTag);
        int length = end - start;
        string rawHandData = rawData.Substring(start, length);

        GetFingerData(h, rawHandData);
        
        //GetSubunitData(JointType.index, rawHandData);
    }

    void GetFingerData(Hand h, string rawHandData)
    {
        h.wrist.rotationValues = GetSubunitData(JointType.wrist, rawHandData);
        h.thumb.rotationValues = GetSubunitData(JointType.thumb, rawHandData);
        h.index.rotationValues = GetSubunitData(JointType.index, rawHandData);
        h.middle.rotationValues = GetSubunitData(JointType.middle, rawHandData);
        h.ring.rotationValues = GetSubunitData(JointType.ring, rawHandData);
        h.pinky.rotationValues = GetSubunitData(JointType.pinky, rawHandData);

    }

    Vector3 GetSubunitData(JointType j, string rawHandData)
    {
        if (j == JointType.leftHand || j == JointType.rightHand) return Vector3.zero;
        string startTag = GetStartTag(j);
        string endTag = GetEndTag(j);
        int start = rawHandData.IndexOf(startTag) + startTag.Length;
        int end = rawHandData.IndexOf(endTag);
        int length = end - start;
        string rawSubunitData = rawHandData.Substring(start, length);

        string[] cleanedData = rawSubunitData.Split(separators, StringSplitOptions.RemoveEmptyEntries);
        float[] vectArray = Array.ConvertAll(cleanedData, float.Parse);
        return new Vector3(vectArray[0], vectArray[1], vectArray[2]);
        //return Vector3.zero;
    }

    string GetStartTag(JointType jointType)
    {
        string startTag = "";
        switch (jointType)
        {
            case JointType.thumb:
                startTag = "<thumb>";
                break;
            case JointType.index:
                startTag = "<index>";
                break;
            case JointType.middle:
                startTag = "<middle>";
                break;
            case JointType.ring:
                startTag = "<ring>";
                break;
            case JointType.pinky:
                startTag = "<pinky>";
                break;
            case JointType.leftHand:
                startTag = "<leftHand>";
                break;
            case JointType.rightHand:
                startTag = "<rightHand>";
                break;
            case JointType.wrist:
                startTag = "<wrist>";
                break;
        }

        return startTag;
    }

    string GetEndTag(JointType jointType)
    {
        string endTag = "";
        switch (jointType)
        {
            case JointType.thumb:
                endTag = "</thumb>";
                break;
            case JointType.index:
                endTag = "</index>";
                break;
            case JointType.middle:
                endTag = "</middle>";
                break;
            case JointType.ring:
                endTag = "</ring>";
                break;
            case JointType.pinky:
                endTag = "</pinky>";
                break;
            case JointType.leftHand:
                endTag = "</leftHand>";
                break;
            case JointType.rightHand:
                endTag = "</rightHand>";
                break;
            case JointType.wrist:
                endTag = "</wrist>";
                break;
        }

        return endTag;
    }

    void UpdateHand()
    {
        
    }
}