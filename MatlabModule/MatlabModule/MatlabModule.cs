using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.IO;
using System.Text;
using KSP.IO;
using System.Collections.Generic;

public class MatlabModule : PartModule
{
    public string connectionIP = "127.0.0.1";
    public int connectionPort = 25001;
    public static int flag = 0;
    public static TcpClient tcpclnt;
    public static FlightCtrlState s_ext = new FlightCtrlState();


    public override void OnStart(StartState state)
    {

        //BinaryReader br;

        if(state != StartState.Editor)
        {

            try
            {
                tcpclnt = new TcpClient();
                print("Connecting.....");

                tcpclnt.Connect("127.0.0.1", 25001);
                // use the ipaddress as in the server program

                print("Connected");
       
                                     


            }

            catch (Exception e)
            {
                print("Error..... " + e.StackTrace);
            }

            
            
            // *** Set Action For Destruction ***
            this.vessel.OnJustAboutToBeDestroyed += CleanUp;
            this.part.OnJustAboutToBeDestroyed += CleanUp;
            
                                               
        }       
    }
    

    public override void OnUpdate()
    {
        Stream stm = tcpclnt.GetStream();

        System.IO.BinaryReader binReader = new System.IO.BinaryReader(stm);
        System.IO.BinaryWriter binWriter = new System.IO.BinaryWriter(stm);

        byte retcode = 42;
        int cmd;
        

        if((stm.CanRead) && (tcpclnt.Available > 0))
        {
            // *** Get Command *** 
            cmd = (int)binReader.ReadByte();
            
            // *** Take Action Based On Command Code ***
            switch(cmd)
            {
                // ----- Comm Actions -----
                case 0:
                    binWriter.Write(retcode);
                    break;
                case 1:
                    binWriter.Write(retcode);
                    break;

                // ----- Vessel Information -----
                case 15:
                    //Gyro Rates
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.angularVelocity.x);
                    binWriter.Write(this.vessel.angularVelocity.y);
                    binWriter.Write(this.vessel.angularVelocity.z);
                    break;

                case 16:
                    // Position
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.orbit.pos.x);
                    binWriter.Write(this.vessel.orbit.pos.y);
                    binWriter.Write(this.vessel.orbit.pos.z);
                    print("X position = " + this.vessel.orbit.pos.x);
                    break;

                case 17:
                    // Velocity
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.orbit.vel.x);
                    binWriter.Write(this.vessel.orbit.vel.y);
                    binWriter.Write(this.vessel.orbit.vel.z);
                    break;

                case 18:
                    // Surface Relative Quaternion Rotation
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.srfRelRotation.w);
                    binWriter.Write(this.vessel.srfRelRotation.x);
                    binWriter.Write(this.vessel.srfRelRotation.y);
                    binWriter.Write(this.vessel.srfRelRotation.z);
                    break;

                case 19:
                    // Euler Angles
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.srfRelRotation.eulerAngles.x);
                    binWriter.Write(this.vessel.srfRelRotation.eulerAngles.y);
                    binWriter.Write(this.vessel.srfRelRotation.eulerAngles.z);                    
                    break;

                case 20:
                    // Angular Velocities
                    binWriter.Write(retcode);
                    binWriter.Write(32.5F);
                    break;

                case 21:
                    // Mission Time
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.missionTime);
                    break;

                case 22:
                    //
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.rigidbody.position.x);
                    binWriter.Write(this.vessel.rigidbody.position.y);
                    binWriter.Write(this.vessel.rigidbody.position.z);
                    break;

                case 23:
                    // Orbiting Body Position
                    binWriter.Write(retcode);
                    while (tcpclnt.Available < 1) { }
                    Int32 body_index = binReader.ReadInt32();
                    List<CelestialBody> bodies = this.vessel.mainBody.orbitingBodies;
                    if (body_index > bodies.Count)
                    {
                        double x = 0;
                        double y = 0;
                        double z = 0;
                        binWriter.Write(x);
                        binWriter.Write(y);
                        binWriter.Write(z);
                    }
                    else
                    {
                        CelestialBody body = bodies[body_index];
                        binWriter.Write(body.orbit.pos.x);
                        binWriter.Write(body.orbit.pos.y);
                        binWriter.Write(body.orbit.pos.z);
                    }                        
                    break;

                case 24:
                    // Vessel Mass
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.GetTotalMass());
                    break;

                case 25:
                    // Forward Vector
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.GetFwdVector().x);
                    binWriter.Write(this.vessel.GetFwdVector().y);
                    binWriter.Write(this.vessel.GetFwdVector().z);
                    break;

                case 26:
                    // Surface Relative Velocity
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.GetSrfVelocity().x);
                    binWriter.Write(this.vessel.GetSrfVelocity().y);
                    binWriter.Write(this.vessel.GetSrfVelocity().z);
                    break;

                case 27:
                    // Quaternion Rotation
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.ReferenceTransform.rotation.w);
                    binWriter.Write(this.vessel.ReferenceTransform.rotation.x);
                    binWriter.Write(this.vessel.ReferenceTransform.rotation.y);
                    binWriter.Write(this.vessel.ReferenceTransform.rotation.z);
                    break;

                case 28:
                    // Rigid Body Velocity
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.orbit.GetWorldSpaceVel().x);
                    binWriter.Write(this.vessel.orbit.GetWorldSpaceVel().y);
                    binWriter.Write(this.vessel.orbit.GetWorldSpaceVel().z);
                    break;

                case 29:
                    // Reference Body Velocity
                    binWriter.Write(retcode);
                    binWriter.Write(this.vessel.mainBody.orbit.GetWorldSpaceVel().x);
                    binWriter.Write(this.vessel.mainBody.orbit.GetWorldSpaceVel().y);
                    binWriter.Write(this.vessel.mainBody.orbit.GetWorldSpaceVel().z);
                    break;

                // ----- Control Imperatives -----
                case 32:
                    // On/Off Autopilot
                    binWriter.Write(retcode);
                    while (tcpclnt.Available < 1) { } 
                    if (binReader.ReadByte() == 1)
                    {
                        //Initialize Flight Control State
                        s_ext.roll = 0F;            //zero roll
                        s_ext.pitch = 0F;           //zero pitch
                        s_ext.yaw = 0F;             //zero yaw
                        s_ext.mainThrottle = 0F;    //zero main throttle 
                        //Engage Autopilot
                        this.vessel.OnFlyByWire += new FlightInputCallback(fly);
                        print("Matlab Piloting enabled.");
                    }
                    else
                    {
                        //Disengage autopilot
                        this.vessel.OnFlyByWire -= new FlightInputCallback(fly);
                        print("Matlab Piloting enabled.");
                    }
                    break;

                case 33:
                    // Roll, Pitch, Yaw   
                    binWriter.Write(retcode);
                    while (tcpclnt.Available < 12) { }                   
                    s_ext.roll = binReader.ReadSingle();            //zero roll
                    s_ext.pitch = binReader.ReadSingle();           //zero pitch
                    s_ext.yaw = binReader.ReadSingle();             //zero yaw                    
                    break;
                                        
                case 34:
                    // Primary Thrust
                    binWriter.Write(retcode);
                    while (tcpclnt.Available < 4) { }
                    s_ext.mainThrottle = binReader.ReadSingle();
                    break;

                case 35:
                    // Trigger Next Stage
                    binWriter.Write(retcode);
                    Staging.ActivateNextStage();
                    break;

                case 36:
                    // Thruster Translation
                    binWriter.Write(retcode);
                    while (tcpclnt.Available < 12) { }                   
                    s_ext.X = binReader.ReadSingle();            //zero roll
                    s_ext.Y = binReader.ReadSingle();           //zero pitch
                    s_ext.Z = binReader.ReadSingle();             //zero yaw
                    break;

                case 37:
                    // Raise/Lower Landing Gear
                    binWriter.Write(retcode);
                    
                    break;

                case 38:
                    // Lights
                    binWriter.Write(retcode);                    
                    break;

                case 39:
                    // Solar Panels
                    binWriter.Write(retcode);
                    break;


                // ----- No Specified Command -----
                default:
                    binWriter.Write((byte)0);
                    break;

            }
                                    
        }
           

    }

    private void CleanUp()
    {
        print("clean-up has occured");
        tcpclnt.Close();
    }


    //now this function gets called every frame or something and gives you access to the flight controls
    private void fly(FlightCtrlState s)
    {
        s.yaw = s_ext.yaw;  //set yaw input
        s.pitch = s_ext.pitch; //set pitch input to whatever the player has input + 30%
        s.roll = s_ext.roll;   //set roll to 50% (either clockwise or counterclockwise, try it and find out)
        s.mainThrottle = s_ext.mainThrottle; //set throttle to 80%

        //the range of yaw, pitch, and roll is -1.0F to 1.0F, and the throttle goes from 0.0F to 1.0F.
        //if your code might violate that it's probably a good idea to clamp the inputs, e.g.:
        //s.roll = Mathf.Clamp(s.roll, -1.0F, +1.0F);
    }

      

     


}



