/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Jaume Nin <jaume.nin@cttc.cat>
 */

//UNCONSTRAINED Q-learning Problem
//LTE includes
#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/nstime.h"
#include "ns3/netanim-module.h"
//Wifi Includes
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/csma-module.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>

#include "ns3/packet-sink.h"
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

using namespace ns3;

/* Sample simulation script for LTE+EPC. It instantiates one eNodeB,
 * attaches one UE to that eNodeB, starts a flow for each UE from a remote host.
 * Half of the users are voice users, half are data users. 
 * Wifi added with it. each wifi user downloads a file from AP.*/

 NS_LOG_COMPONENT_DEFINE ("EpcFirstExample");
 std::ofstream macFile;
 std::ofstream macFile1;
 std::ofstream macFile2;
 std::ofstream macFile3;
std::ofstream macFile4;
 /***********common parameters**************/
  
  uint16_t numberOfNodes = 10;//no of UEs
  //To keep track of current no of users in different RATs
  uint16_t voiceuser=0;
  uint16_t datauser_lte=0;
  uint16_t datauser_wifi=0;
  // To store the value of thresholds (C+W) dimensional vector
  double threshold[18];

  //To keep track of no of users in different RATs and action in last state
  uint16_t voiceuser_old=0;
  uint16_t datauser_lte_old=0;
  uint16_t datauser_wifi_old=0;
  uint16_t action_old=0;
  uint16_t index_old=0;
  double mbps_periodic=0;
  double mbps_event=0;
  double total_interval=0;

  //To keep track of fast and slow-time scale
  uint32_t iteration_fast=1;
  uint32_t iteration_slow=1;
  uint32_t iteration_throughput=0;
  double sum_step=0;
  //Total number of bytes received using tracing
  //static uint32_t m_bytesTotal;
  static uint32_t m_totalRx;
  double mbps,mbps1;
  //keep track of last time throughput() function was called
  Time old_time;
  uint16_t first_time=0;
  uint16_t first_event=0;
  //To calculate the time of stay in a state
  double time_of_stay=0;
  //Lagrange multiplier for cost
  double beta=0;
  double beta_array[500000];//updating beta values
  double time_array[500000];//time of updating beta array
  //To identify request maker
  uint16_t handle=0;
  // Busy-ness of different servers
  uint16_t busy_lte[10];//common resources for data and voice in LTE. Set =1 for voice, =2 for data
  uint16_t busy_data_wifi[8];
  double report_LTE[10],report_wifi[8];// To store SNR in LTE and WiFi
  double inv_snr_wifi,inv_snr_LTE;
  uint16_t index_snr; // To store the index of user with min SNR


  double   state_PDS_value[11];// To store PDS values for different states
  uint32_t state_PDS_visit[9][11][11];// Number of visits to different states and actions
  uint16_t feasible_action_counter[9][11][11][6];// To store number of feasible actions in a state
  uint16_t feasible_action_set[9][11][11][6];// To store feasible actions in concatenated form e.g:123 etc
  // double   avg_cost=0; // To store average cost (may not be necessary)
  uint16_t optimal_action[9][11][11][6];//to store optimal action

 //Start and stop time of different servers
  double startarray_voice=0;
  double endarray_voice=0;
  double startarray_data=0;
  double endarray_data=0;

  //To generate random arrival and departure time
  double starttime_voice=0;//start time of voice users
  double endtime_voice= 0;//end time of voice users
  double starttime_data=0;//start time of data users
  double endtime_data= 0;//end time of data users
  
 //voice and data user arrival and service rate
  double onebylambdav=1;
  double onebylambdad=1;
  double onebymuv=0.25;
  double onebymud=0.25;
  
 //To keep track of blocking
  double block=0;//(blocking in non-blocking states)
  double block_all=0;// blocking in all states
  static uint32_t flowcount=0;//to count the number of flow
  uint16_t flowstat[300000];// store flow statistics
  double simTime =3000;//Simulation time
 //create application to send data
  class MyApp : public Application
  {
  public:
  
    MyApp ();
    virtual ~MyApp();
  
    void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, 
      /*uint32_t nPackets*/DataRate dataRate);
  
  
    virtual void StartApplication (void);
    virtual void StopApplication (void);
 // private:
    void ScheduleTx (void);
    void SendPacket (void);
  
    Ptr<Socket>     m_socket;
    Address         m_peer;
    uint32_t        m_packetSize;
    //uint32_t        m_nPackets;
    DataRate        m_dataRate;
    EventId         m_sendEvent;
    bool            m_running;
    uint32_t        m_packetsSent;
  };

//Arrays to hold apps running in different nodes
  ApplicationContainer sourceApps_voice[10];
  ApplicationContainer sinkApps_voice[10];
  ApplicationContainer sourceApps_data_lte[10];
  ApplicationContainer sinkApps_data_lte[10];
  ApplicationContainer sourceApps_data_wifi[8];
  ApplicationContainer sinkApps_data_wifi[8];

//Declaration
  NodeContainer ueNodes;
  NodeContainer remoteHostContainer;
  Ptr<Node> remoteHost;
  Ipv4InterfaceContainer ueIpIface,staIpIface;
  Ipv4InterfaceContainer staInterfaces,apInterfaces;
  Ipv4InterfaceContainer internetIpIfaces;
  NodeContainer wifiStaNodes,LteStaNodes;
  NodeContainer wifiApNode;
  uint16_t dlPort = 1234;
  uint16_t port = 9;  // well-known echo port number

  uint16_t found_data_stop;
  uint16_t action;
  
  void MyCallbackArrival_data_lte(double,double);
  void MyCallbackArrival_data_wifi(double,double);
  //void After_PDS_learning(uint16_t,double);
  void PDS_learning (uint16_t,double,double);
  void structured_PDS_learning (uint16_t,double,double);
  void policy_choice_data (double, double);
  void policy_choice_voice (double, double);
  double Throughput ();
//  void Periodic_Throughput();
  double Threshold_value(uint16_t);
  void Event_driven_throughput();


 //Constructor and destructor of my app
  MyApp::MyApp ()
    : m_socket (0),
      m_peer (),
      m_packetSize (0),
      //m_nPackets (0),
      m_dataRate (0),
      m_sendEvent (),
      m_running (false),
      m_packetsSent (0)
  {
  }
  
  MyApp::~MyApp()
  {
    m_socket = 0;
  }

 //configuration of my app
 void MyApp::Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, 
                     /*uint32_t nPackets*/ DataRate dataRate)
{
  m_socket = socket;
  m_peer = address;
  m_packetSize = packetSize;
 // m_nPackets = nPackets;
  m_dataRate = dataRate;
}

//method to start my app
void MyApp::StartApplication (void)
  {
    m_running = true;
    m_packetsSent = 0;
    m_socket->Bind ();
    m_socket->Connect (m_peer);
    SendPacket ();
  }

//method to stop my app
void MyApp::StopApplication (void)
  {
    m_running = false;
  
    if (m_sendEvent.IsRunning ())
      {
        Simulator::Cancel (m_sendEvent);
      }
  
    if (m_socket)
      {
        m_socket->Close ();
      }
  }

//method to send packet
void MyApp::SendPacket (void)
  {
    Ptr<Packet> packet = Create<Packet> (m_packetSize);
    m_socket->Send (packet);
  
    //if (++m_packetsSent < m_nPackets)
      {
        ScheduleTx ();
      }
  }

//method to schedule transmission
void MyApp::ScheduleTx (void)
  {
    if (m_running)
      {
        Time tNext (Seconds (m_packetSize * 8 / static_cast<double> (m_dataRate.GetBitRate ())));
        m_sendEvent = Simulator::Schedule (tNext, &MyApp::SendPacket, this);
      }
  }

//Tracing for correctly received packet (IPV4) packet sink didn't work
//static void
//ReceivePacket(std::string context, Ptr<Packet const> packet, Address const& add)
void IPv4ReceivedPackets(std::string context,const Ptr< const Packet > packet, Ptr<Ipv4 > ipv4, const uint32_t interface)
{

    Ptr<Packet> m_currentPacket;
    m_currentPacket = packet->Copy();
 
    m_totalRx += m_currentPacket->GetSize ();
   
} 

/****************** Obtain uid from context ***************/

uint32_t
GetUidFromContext (const std::string context)
{
  uint16_t delta = context.substr(10).find_first_of("/");
  std::stringstream ss;
  uint32_t uid;
  ss.str (context.substr(10, delta));
  ss >> uid;
  return uid;
}

//LTE RSRP reading
void
ReportCurrentCellRsrpSinr(std::string context, uint16_t cellId, uint16_t rnti, double rsrp, double sinr)
{
uint32_t uid=GetUidFromContext(context);
if (uid>=13 && uid<=22)
{
 report_LTE[uid-13]=sinr;
 //std::cout << "ue id:" << uid << "LTE sinr:"<< sinr <<"\n";
}
}

//WiFi RSSI reading
void
PhyRxOkTrace (std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble)
{
uint32_t uid=GetUidFromContext(context);
if (3<=uid && uid<=10)
{
 report_wifi[uid-3]=snr;
 //std::cout << "ue id:" << uid << "WiFi snr:"<< snr <<"\n";
}
}






//calculation of throughput for every event
void Event_driven_throughput()
{
 if (first_event==10)
{
  Time time = Simulator::Now ();
   double interval=time_of_stay;
   total_interval=total_interval+interval;
  std::cout << "event driven interval:" << interval << "\n";
  mbps_event=(mbps1+(total_interval-interval)*mbps_event)/((total_interval));//yo
  sum_step=sum_step+pow(floor(iteration_throughput*0.01)+2,-0.6);
   std::cout << "step size:" << (sum_step)/(iteration_throughput+1) << "\n";
//mbps_periodic = ((m_totalRx * 8.0) / 1000000)/(interval);
 // mbps = ((m_totalRx * 8.0) / 1000000)/(10.0*throughput_iteration);//remembar to remove
  std::cout << time.GetSeconds() << "Event driven throughput"<<" " << mbps_event << "Rxbytes"<<m_totalRx<<"iteration"<<iteration_throughput<<"\n";
  macFile4 << iteration_throughput <<" "<< mbps_event<< " " << sum_step << " " <<"\n";
  //std::cout << time.GetSeconds() <<"old time"<< " " << old_time.GetSeconds()<< "\n";
  m_totalRx = 0;
  //Simulator::Schedule (Seconds (2.0), &Throughput);
 //print the value of thresholds
   for (uint32_t thue=0;thue<18;thue++)  
   {
      std::cout << "threshold value:" << threshold[thue] << "\n";
   }
  iteration_throughput++;

 }
else
 first_event=10;
}


// Calculation of throughput using trace + include cost also
double Throughput (int index_old, int action_old) // in Mbps calculated whenever some events occur
{
  Time time = Simulator::Now ();
 // double mbps=0;
  time_of_stay=time.GetSeconds()-old_time.GetSeconds();

  //mbps = ((m_totalRx * 8.0) / 1000000)/(time.GetSeconds()-old_time.GetSeconds());//remembar to remove
  
  macFile2 << time.GetSeconds() <<"time"<< " " << time.GetSeconds()<< "\n";
  macFile2 << time.GetSeconds() <<"old time"<< " " << old_time.GetSeconds()<< "\n";
  //m_totalRx = 0;
  //Simulator::Schedule (Seconds (2.0), &Throughput);
  //old_time=Simulator::Now();
  
 
if (index_old==1 && action_old==1)
{
   mbps=-pow(voiceuser_old+datauser_lte_old,2)*0.1+state_PDS_value[voiceuser_old+datauser_lte_old];
   mbps1=-pow(voiceuser_old+datauser_lte_old,2)*0.1;
}
if (index_old==1 && action_old==2)
{
   mbps=-pow(voiceuser_old+datauser_lte_old,2)*0.1+20+state_PDS_value[voiceuser_old+1+datauser_lte_old];
   mbps1=-pow(voiceuser_old+datauser_lte_old,2)*0.1+20;
}

if (index_old==2 && action_old==1)
{
   mbps=-pow(voiceuser_old+datauser_lte_old,2)*0.1+state_PDS_value[voiceuser_old+datauser_lte_old];
   mbps1=-pow(voiceuser_old+datauser_lte_old,2)*0.1;
}
if (index_old==2 && action_old==2)
{
   mbps=-pow(voiceuser_old+datauser_lte_old,2)*0.1+10+state_PDS_value[voiceuser_old+1+datauser_lte_old];
   mbps1=-pow(voiceuser_old+datauser_lte_old,2)*0.1+10;
}


return mbps;
}

double Threshold_value (uint16_t datauser_wifi)
{
    
   //if (datauser_wifi==0)
    {
      if (drand48()<0.5)
        return  state_PDS_value[datauser_lte+voiceuser+1];
      else
        return -state_PDS_value[datauser_lte+voiceuser];
    }
                
    
}


//callback function for voice departure
void MyCallbackDep_voice (uint16_t handle) 
{
    macFile2 << "voice departure" << std::endl;
    macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
    //double t;
    //uint16_t cardinality_action=0,concat_action=0,action=0;
  

     //releasing resource after departure
     if (busy_lte[handle]==1)
     {
        busy_lte[handle]=0;
        voiceuser--;
     }
        voiceuser_old=voiceuser;
} 
        
//callback function for voice departure
void MyCallbackDep_data_lte (uint16_t handle) 
{
    macFile2 << "Data departure" << std::endl;
    macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
    //double t;
    //uint16_t cardinality_action=0,concat_action=0,action=0;
  

     //releasing resource after departure
     if (busy_lte[handle]==2)
     {
        busy_lte[handle]=0;
        datauser_lte--;
     }
        datauser_lte_old=datauser_lte;
} 
         

//callback function for voice arrival
void MyCallbackArrival_voice (double stime, double etime, uint16_t action) 
{
  macFile2 << "voice arrival" << std::endl;
  macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
  uint16_t found_lte_voice=0;
  found_data_stop=0;
  inv_snr_LTE=0;// find UE with min SNR or max inv_SNR

//if action=4, offload one data user from LTE to WiFi
  for (uint16_t u=0;u<10;u++)
 {
  if (busy_lte[u]==0 && found_lte_voice==0)
  {
        found_lte_voice=1;
        voiceuser++;
        Ptr<Socket> ns3UdpSocket = Socket::CreateSocket (ueNodes.Get (u),UdpSocketFactory::GetTypeId ());
        Address sinkAddress (InetSocketAddress(internetIpIfaces.GetAddress (1), dlPort));//Interface betn pgw and RH
        PacketSinkHelper sink ("ns3::UdpSocketFactory", sinkAddress);
        sinkApps_voice[u] = sink.Install (remoteHost);
        sinkApps_voice[u].Start (Seconds (0.0));
        sinkApps_voice[u].Stop (Seconds(etime-stime));
        Ptr<MyApp> app = CreateObject<MyApp> ();
        app->Setup (ns3UdpSocket, sinkAddress, 50,  DataRate ("20kbps"));
        ueNodes.Get (u)->AddApplication (app);
        sourceApps_voice[u].Add(app);
        app->SetStartTime (Seconds (0.0));
        app->SetStopTime (Seconds(etime-stime));
        dlPort = dlPort + 1;
        busy_lte[u]=1;
        flowcount++;
        flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);
        macFile2 << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << " \n";//added
        //schedule a departure callback function
        Simulator::Schedule(Seconds(etime-stime), MyCallbackDep_voice,u);
     }
  }

 
}


//callback function for data arrival in lte
void MyCallbackArrival_data_lte (double stime, double etime) 
{
  macFile2 << "data arrival in lte" << std::endl;
  macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
  uint16_t found_lte=0;
  for (uint16_t u=0;u<10;u++)
  {
    if (busy_lte[u]==0 && found_lte==0)
     {
        found_lte=1;
        datauser_lte++;
        Ptr<Socket> ns3UdpSocket = Socket::CreateSocket (LteStaNodes.Get (u),UdpSocketFactory::GetTypeId ());
        
        Address sinkAddress (InetSocketAddress(internetIpIfaces.GetAddress (1), dlPort));//Interface betn pgw and RH

        PacketSinkHelper sink ("ns3::UdpSocketFactory", sinkAddress);
        sinkApps_data_lte[u] = sink.Install (remoteHost);//sink in remotehost
        sinkApps_data_lte[u].Start (Seconds (0.0));
        sinkApps_data_lte[u].Stop (Seconds(etime-stime));
        Ptr<MyApp> app = CreateObject<MyApp> ();
        app->Setup (ns3UdpSocket, sinkAddress, 600,  DataRate ("5Mbps"));
        LteStaNodes.Get (u)->AddApplication (app);
        sourceApps_data_lte[u].Add(app);
        app->SetStartTime (Seconds (0.0));
        app->SetStopTime (Seconds(etime-stime));//mod
        dlPort = dlPort + 1;
        busy_lte[u]=2;
        flowcount++;
        flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);//how many users related to this flow
        macFile2<< " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << " \n";//added
        Simulator::Schedule(Seconds(etime-stime), MyCallbackDep_data_lte,u);
   }
  
}
}


void policy_choice_voice(double stime, double etime)
{
   
macFile2 << " flowcount=: " <<flowcount<< " \n";//added
//if (first_time==0)
  {
        //first_time=10;  
        voiceuser_old=voiceuser;
        datauser_lte_old=datauser_lte;
        datauser_wifi_old=datauser_wifi;
        index_old=1;
        if(first_time==0)
        {        old_time=Simulator::Now ();//update old_time for the first time
                first_time=10;
        }

        structured_PDS_learning(1, stime, etime);
  }

}


void policy_choice_data(double stime, double etime)
{
   
macFile2 << " flowcount=: " <<flowcount<< " \n";//added
//if (first_time==0)
  {
        //first_time=10;  
        voiceuser_old=voiceuser;
        datauser_lte_old=datauser_lte;
        datauser_wifi_old=datauser_wifi;
        index_old=2;
        //std::cout << "policy choice voice" << std::endl;
        if(first_time==0)
        {        old_time=Simulator::Now ();//update old_time for the first time
                first_time=10;
        }

        structured_PDS_learning(2, stime, etime);
  }

}


// structured learning for voice user arrival
void structured_PDS_learning (uint16_t index,double stime, double etime)
{
        //double p,s=0,t;//reward=0;
        double t;
        //uint16_t cardinality_action=0,concat_action=0,
        uint16_t action=0;
        //slow time scale
        double rho1=10*pow(iteration_slow,-1);
        
               macFile2 <<   "exploitation \n";
               
               //when datauser=0 in wifi, only 1 threshold, datauser in LTE is zero
               if (index==1)
               {
                  if ((voiceuser+datauser_lte)<threshold[0])
                  {
                     action=2;
                  }
                  else
                  {
                    action=1;
                  }
                  //multiplier for the scheme, approximation using this function
                  double mul=exp(voiceuser+datauser_lte-threshold[0]-0.5)/pow((1+exp(voiceuser+datauser_lte-threshold[0]-0.5)),2);
                     threshold[0]=threshold[0]-mul*rho1*Threshold_value(0);
       
                if (threshold[0]<=0)//Projection operator
                   threshold[0]=0;
                if (threshold[0]>10)
                   threshold[0]=10;
                if (threshold[1]>threshold[0])
                   threshold[1]=threshold[0];
                }


             if (index==2)
               {
                  if ((voiceuser+datauser_lte)<threshold[1])
                  {
                     action=2;
                  }
                  else
                  {
                    action=1;
                  }
                  //multiplier for the scheme, approximation using this function
                  double mul=exp(voiceuser+datauser_lte-threshold[1]-0.5)/pow((1+exp(voiceuser+datauser_lte-threshold[1]-0.5)),2);
                  //Update the appropriate threshold value
                     threshold[1]=threshold[1]-mul*rho1*Threshold_value(0);
                  
       
                if (threshold[1]<=0)//Projection operator
                   threshold[1]=0;
                if (threshold[1]>threshold[0])
                   threshold[1]=threshold[0];
                }
               
               t= Throughput(index_old,action);
               Event_driven_throughput();
               old_time=Simulator::Now();
               macFile2 << "exploit action4: " << action << "\n";
               optimal_action[datauser_wifi][voiceuser][datauser_lte][index]=action;
     
        state_PDS_visit[datauser_wifi][voiceuser][datauser_lte]++;
       
if (index==1)
{
 if (action==2)
  {   
      Simulator::Schedule(Seconds(0.0), MyCallbackArrival_voice,stime,etime,2);
  }
  
  else 
      {
               macFile2 << "voice user is blocked" << std::endl;
               std::cout << "voice user is blocked" << std::endl;
               block_all++;
               if ((voiceuser+datauser_lte)!=10)
               {
                block++;//Count no. of blocked voice calls except states where only possible action is blocking
               macFile2 << "blocked calls: " <<block<< "\n";
          
              }
      }
}

if (index==2)
{
 if (action==2)
  {   
      Simulator::Schedule(Seconds(0.0), MyCallbackArrival_data_lte,stime,etime);
  }
  
  else 
      {
               macFile2 << "data user is blocked" << std::endl;
               std::cout << "data user is blocked" << std::endl;
               block_all++;
               if ((voiceuser+datauser_lte)!=10)
               {
                block++;//Count no. of blocked voice calls except states where only possible action is blocking
               macFile2 << "blocked calls: " <<block<< "\n";
          
              }
      }
}


 
 double cost;
double rho=pow(floor(iteration_fast*0.01)+2,-0.6);//global clock
//fast time sacle
  //double rho=pow(iteration_fast,-0.7);
// beta is necessary only for CMDP problems (LM)
  macFile2 << "iteration fast:" << iteration_fast<<" \n"; 
  iteration_fast=iteration_fast+1;
  macFile2 << "time of stay:" << time_of_stay<<" \n"; 
  macFile2<< "iteration slow:" << iteration_slow<<" \n"; 
  iteration_slow=iteration_slow+1;
 if (index_old==1 && action==1)
        cost=1;
 else
        cost=0;
  beta=beta+rho1*(cost-0.05); //
  if (beta<=0)//Projection operator
     beta=0;
  if (beta>100000)
     beta=100000;
  beta_array[iteration_slow]=beta;
Time o_time = Simulator::Now ();
  time_array[iteration_slow]=o_time.GetSeconds();
 macFile2 << "beta updated:" << beta<<" \n"; 

  state_PDS_value[voiceuser_old+datauser_lte_old]=(1-rho)*state_PDS_value[voiceuser_old+datauser_lte_old];
 
  state_PDS_value[voiceuser_old+datauser_lte_old]+=rho*(t-state_PDS_value[0]*1);

}

int main (int argc, char *argv[])
{
  /*************LTE parameters**********/
  //uint16_t numberOfNodes = 4;//no of UEs
  uint16_t numberOfBS=1;//no of BS
  //double simTime =150;//Simulation time
  double distance = 100.0;//distance between BSs and UEs
  double radius = 100.0;//radius of cell
  double interPacketInterval = 100;//what abt making this random?
  //uint32_t maxbytes=10000;//added amount of data to be transferred to each client

 /**********Wifi parameters*************/
  bool verbose = true;
  bool enableFading = true; //yes fading        
  uint32_t nWifi = 10;
  //code added to configure TCP
  //Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (2008));
  //Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (9000000));
  //Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (9000000));
   // WIFI Nodes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("10000"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("10000"));
  
 /***********3GPP Parameters*****************/
  double bsAntennaHeight = 32.0;
  double msAntennaHeight = 1.5;
  double apAntennaHeight = 2.5;
  Config::SetDefault ("ns3::LteEnbPhy::TxPower",  DoubleValue (46));
  Config::SetDefault ("ns3::LteEnbPhy::NoiseFigure",  DoubleValue (5));
  Config::SetDefault ("ns3::LteUePhy::TxPower",  DoubleValue (23));
  Config::SetDefault ("ns3::LteUePhy::NoiseFigure",  DoubleValue (9));
  //Config::SetDefault ("ns3::ApWifiMac::EnableBeaconJitter", BooleanValue (true));
  //Config::SetDefault ("ns3::StaWifiMac::AutoAssociation", BooleanValue (false));  // Manual association
 
  /*********** Command line arguments***************/
  CommandLine cmd;
  cmd.AddValue("numberOfNodes", "Number of UEs", numberOfNodes);
  cmd.AddValue("numberOfBSs", "Number of BSs", numberOfBS);
  cmd.AddValue("simTime", "Total duration of the simulation [s])", simTime);
  cmd.AddValue("distance", "Distance between eNBs [m]", distance);
  cmd.AddValue ("radius", "the radius of the disc where UEs are placed around an eNB", radius);
  cmd.AddValue("interPacketInterval", "Inter packet interval [ms])", interPacketInterval);
  // Command line arguments for Wifi
  cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.Parse(argc, argv);

  macFile.open ("vt_struc.txt");
  macFile << "iteration\tbeta\n";
  macFile1.open ("vt_stat_struc.txt");
  macFile1 << "voiceuser\t data_lte \t data_wifi \t index \t opt action \t opt act value \t state action visit\n";
 macFile2.open ("vt_alltrace_struc.txt");
 macFile3.open ("vt_thr_struc.txt");
macFile4.open ("vt_event_thr_struc.txt");

   if (verbose)
    {
      //LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
      //LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

/***************Initialization*****************/
  for(uint16_t i=0;i<numberOfNodes;i++)
    busy_lte[i]=0;

  for(uint16_t i=0;i<8;i++)
    busy_data_wifi[i]=0;

 for(uint16_t i=0;i<11;i++)
    state_PDS_value[i]=0;

  uint16_t count1=0;

//Initialize action set and action counter for states
//Initialize optimal action set too
  for (uint16_t i=0;i<9;i++)
  {
        for (uint16_t j=0;j<11;j++)
        {
                for (uint16_t k=0;k<11;k++)
                {
                        if ((j+k) < 11)
                        {
                                for (uint16_t index=1;index<6;index++)
                                {
                                        feasible_action_counter[i][j][k][index]=0;
                                        feasible_action_set[i][j][k][index]=0;
                                        optimal_action[i][j][k][index]=0;
                                        
                                }
                        }
                }
        }
   }
 uint16_t i=0,j=0,k=0;
 count1=0;

   //Populate action set and action counter for states         
  for (i=0;i<9;i++)
  {
     for (j=0;j<11;j++)
     {
        for (k=0;k<11;k++)
       {
         if ((j+k) < 11)
         {
           for (uint16_t index=1;index<6;index++)
          {
     
             //Sturctural property for data users

             if ((index==1)||((index==2)&&((j+k)==10)&&(i==8))||((index==3)&&(((j+k)==10)||(i<2)))||((index==4)&&(((j+k)==10)||(i<2)))||((index==5)&&((k==0)||(i>0))))
             {
                  feasible_action_set[i][j][k][index]=feasible_action_set[i][j][k][index]*10+1;
                  feasible_action_counter[i][j][k][index]++;
              }
            
           //Sturctural property for data users
            if  (((j+k)<10) && (((index==1)&&((i>0)||(k==0)))||((index==2)&&(i>0))))
            {
             
              feasible_action_set[i][j][k][index]=feasible_action_set[i][j][k][index]*10+2;
              feasible_action_counter[i][j][k][index]++;
           }
         
         count1++;

   }
  }
 }

 }

}
//Initialization of thresholds 
//for(i=0;i<18;i++)
 {
    threshold[0]=5;
    threshold[1]=5;
    threshold[2]=9;
    threshold[3]=8;
    threshold[4]=7;
    threshold[5]=6;
    threshold[6]=5;
    threshold[7]=4;
    threshold[8]=3;
    threshold[9]=2;
    threshold[10]=1;
  
 }  

for(i=11;i<18;i++)
 {
        threshold[i]=10;
       
 }                  
                
                   

   // Positive Policies are loaded into corresponding 3-D arrays
      /******LTE Configuration************/
  //create LTE instances and tell it to use EPC
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  lteHelper->SetSchedulerType("ns3::PfFfMacScheduler"); //Scheduler Proportional fair.
  
 
  /************** Setting Extra Lte Parameters ****************/
  lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (50));//10 MHz:50*180 KHz
  lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (50));//10 MHz:50*180 KHz
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::LogDistancePropagationLossModel"));
  lteHelper->SetPathlossModelAttribute ("ReferenceLoss", DoubleValue (15.3));
  lteHelper->SetPathlossModelAttribute ("Exponent", DoubleValue (3.76));

//  fading traces
  lteHelper->SetFadingModel("ns3::TraceFadingLossModel");
  lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue ("src/lte/model/fading-traces/fading_trace_EPA_3kmph.fad"));
  lteHelper->SetFadingModelAttribute ("TraceLength", TimeValue (Seconds (10.0)));
  lteHelper->SetFadingModelAttribute ("SamplesNum", UintegerValue (10000));
  lteHelper->SetFadingModelAttribute ("WindowSize", TimeValue (Seconds (10.0)));
  lteHelper->SetFadingModelAttribute ("RbNum", UintegerValue (100));
  //L=128.1+37.6log(R), R in km
  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  // parse again so you can override default values from the command line
  cmd.Parse(argc, argv);
   //packet gateway
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

   // Create a single RemoteHost
  remoteHostContainer.Create (1);
  remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);
 //-------------------added start--------------------- 
 /********* Point to point Nodes in wifi between AP and RH********/
  wifiApNode.Create (1);
  Ptr<Node> wifiap = wifiApNode.Get (0);
  InternetStackHelper stack;
  stack.Install (wifiApNode);
  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Mb/s")));
  pointToPoint.SetChannelAttribute ("Delay", TimeValue (Seconds (0.)));
 

  NetDeviceContainer p2pDevices;
  p2pDevices = pointToPoint.Install (remoteHost,wifiap);
  Ipv4AddressHelper address;
  //IP address allocation for AP and RH
  address.SetBase ("2.0.0.0", "255.0.0.0");

//---------------------added end----------------------
  /*************Create the Internet***********************/
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  //p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);

 // NodeContainer wifiStaNodes;
  wifiStaNodes.Create (nWifi);
  LteStaNodes.Create (nWifi);
  //ip address allocation for p-p link betn pgw and remote host
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
   internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  //Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);



 // NodeContainer ueNodes;
  NodeContainer enbNodes;
  enbNodes.Create(numberOfBS);
  ueNodes.Create(numberOfNodes);

  

  /*************** Create wifi phy, channel and mac for WAPs ************/
  //WIFI PHY layer

 Config::SetDefault( "ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue (30.6));
 Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue (3.67));
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();

  if (enableFading)
  {
    channel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel", "m0", DoubleValue (1.0),
                                "m1", DoubleValue (1.0), "m2", DoubleValue (1.0));
  }
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());
  phy.Set("TxPowerStart",DoubleValue(23));
  phy.Set("TxPowerEnd",DoubleValue(23));
  phy.Set("RxNoiseFigure",DoubleValue(0));
  phy.SetChannel (channel.Create ());

  // WIFI MAC layer
  WifiHelper wifi = WifiHelper();
  wifi.SetStandard (WIFI_PHY_STANDARD_80211g); // using 802.11g standard
 
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                              "DataMode",StringValue ("ErpOfdmRate54Mbps"), "ControlMode",StringValue ("ErpOfdmRate6Mbps"));

  WifiMacHelper mac = WifiMacHelper ();

  Ssid ssid = Ssid ("ns-3-ssid");
  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid), "ActiveProbing", BooleanValue (false));

  NetDeviceContainer staDevices;
  staDevices = wifi.Install (phy, mac, wifiStaNodes);

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  //**********//
  NetDeviceContainer apDevices;
  apDevices = wifi.Install (phy, mac, wifiApNode);


  /**************** Install Mobility Model****************/
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> positionAlloc1 = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> positionAlloc2 = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> uePosition = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> WifiStaPosition = CreateObject<ListPositionAllocator> ();
  Ptr<ListPositionAllocator> LteStaPosition = CreateObject<ListPositionAllocator> ();
  for (uint16_t i = 0; i < numberOfNodes; i++)
    {
     // positionAlloc->Add (Vector(distance * i, distance * i, 0));//manual position of UEs

    }

  positionAlloc->Add (Vector(100.0,0, bsAntennaHeight));//position of remote host

  for (uint16_t i = 0; i < numberOfBS; i++)
    {
      positionAlloc1->Add (Vector(distance * (i+1), distance * (i+1), bsAntennaHeight));//position of BSs
    }
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

  lteHelper->SetUeAntennaModelType ("ns3::IsotropicAntennaModel");
  lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
  // Position of UEs attached to eNB 1 inside a radius of 100m
  //No UE or AP within 35 m radius of eNB 
  for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
      
      int v1 = rand() % 200; 
      int v2 = rand() % 200; 
      if (pow(v1-100,2)+pow(v2-100,2)>=pow(35,2) && pow(v1-100,2)+pow(v2-100,2)<=pow(100,2))
         uePosition->Add (Vector(v1, v2, msAntennaHeight));
      else
         i--;
    } 
  mobility.SetPositionAllocator(uePosition);
  mobility.Install(ueNodes);
  mobility.SetPositionAllocator(positionAlloc1);
  mobility.Install(enbNodes);
  mobility.SetPositionAllocator(positionAlloc);
  mobility.Install(remoteHostContainer);

  positionAlloc2->Add (Vector(50.0, 30.0, apAntennaHeight));//position of AP

    for (uint32_t i = 0; i < wifiStaNodes.GetN(); ++i)
    {
 
      int v1 = rand() % 60+20; 
      int v2 = rand() % 60+0; 
       if (pow(v1-50,2)+pow(v2-30,2)<=pow(30,2))
         WifiStaPosition->Add (Vector(v1, v2, msAntennaHeight));
       else
         i--;
     
    } 
  mobility.SetPositionAllocator(WifiStaPosition);

  
  mobility.Install(wifiStaNodes);
 for (uint32_t i = 0; i < LteStaNodes.GetN(); ++i)
    {
     
      int v1 = rand() % 60+20; 
      int v2 = rand() % 60+0; 
      if (pow(v1-50,2)+pow(v2-30,2)<=pow(30,2))
         LteStaPosition->Add (Vector(v1, v2, msAntennaHeight));
      else
         i--;
     
    } 
  mobility.SetPositionAllocator(LteStaPosition);
  mobility.Install(LteStaNodes);
  mobility.SetPositionAllocator(positionAlloc2);
  mobility.Install(wifiApNode);


  /*************Create the Internet CONT.***********************/
  stack.Install (wifiStaNodes);
  stack.Install (LteStaNodes);
  //Ipv4InterfaceContainer staInterfaces;
  
  //ip address allocation for STA and AP
   address.SetBase ("3.0.0.0", "255.0.0.0");
   staInterfaces = address.Assign (staDevices);
   apInterfaces= address.Assign (apDevices);
 

  // Install LTE Devices to the nodes
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);
  NetDeviceContainer staLteDevs = lteHelper->InstallUeDevice (LteStaNodes);
  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  //Ipv4InterfaceContainer ueIpIface;
  //Interface between UE and BS
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));
  //Interface between STA and BS
  staIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (staLteDevs));
  

  // Assign IP address to UEs, and install applications
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
  for (uint32_t u = 0; u < LteStaNodes.GetN (); ++u)
    {
      Ptr<Node> staNode = LteStaNodes.Get (u);
      // Set the default gateway for the STA
     Ptr<Ipv4StaticRouting> staStaticRouting = ipv4RoutingHelper.GetStaticRouting (staNode->GetObject<Ipv4> ());
      staStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  // Attach one UE and one STA per eNodeB
  for (uint16_t i = 0; i < numberOfNodes; i++)
      {
        lteHelper->Attach (ueLteDevs.Get(i), enbLteDevs.Get(0));
      }
  for (uint16_t i = 0; i < nWifi; i++)
      {
        lteHelper->Attach (staLteDevs.Get(i), enbLteDevs.Get(0));

      
        // side effect: the default EPS bearer will be activated
       
      }




  /************************ Install and start applications on UEs and remote host*************/

//---------------voice policy start
   uint32_t voice_count=0, data_count=0;
   for (uint16_t i = 0; i < 14; i++)
   {
     starttime_data= starttime_data+(-onebylambdad*log(drand48())); 
     starttime_data=0;  
    }

   while (endtime_voice <(simTime-50))   
  
   {
   
     starttime_voice= starttime_voice+(-onebylambdav*log(drand48()));//1/lambda_v
     endtime_voice= starttime_voice+ (-onebymuv*log(drand48()));//1/mu_v
    startarray_voice=starttime_voice;
    endarray_voice=endtime_voice;
    
    voice_count++;
    Simulator::Schedule(Seconds(startarray_voice), policy_choice_voice,startarray_voice,endarray_voice);
   }

    while (endtime_data <(simTime-50))//remember to remove
  
    {
  
     starttime_data= starttime_data+(-onebylambdad*log(drand48()));//1/lambda_d
     endtime_data= starttime_data+ (-onebymud*log(drand48()));//1/mu_d
     
  
     startarray_data=starttime_data;
     endarray_data=endtime_data;
    // if (data_count==0)
    {
       // std::cout << " start Time for data: " << (startarray_data) << " \n";//added
       // std::cout << " end Time for data: " << (endarray_data) << " \n";//added
    }
     data_count++; 
     Simulator::Schedule(Seconds(starttime_data),policy_choice_data,startarray_data,endarray_data);
  }

   std::ostringstream oss;
   std::ostringstream oss1;
    oss << "/NodeList/" << wifiApNode.Get (0)->GetId() << "/$ns3::Ipv4L3Protocol/Rx"; // for AP it works as it has only one interface
    oss1 << "/NodeList/" << remoteHost->GetId() << "/$ns3::Ipv4L3Protocol/Rx"; // for AP it works as it has only one interface
    Config::Connect (oss.str(), MakeCallback (&IPv4ReceivedPackets));// IPV4 packets received corectly
    Config::Connect (oss1.str(), MakeCallback (&IPv4ReceivedPackets));

Config::Connect ("/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/LteUePhy/ReportCurrentCellRsrpSinr",
                   MakeCallback (&ReportCurrentCellRsrpSinr));//RSRP reporting in LTE
Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/RxOk",
                   MakeCallback (&PhyRxOkTrace));// RSSI reporting in WiFi



//------data policy end
std::cout << " end Time for voice: " << (endarray_voice) << " \n";//added
std::cout << " end Time for data: " << (endarray_data) << " \n";//added
std::cout << " no of voice sessions: " << (voice_count) << " \n";//added
std::cout << " no of data sessions: " << (data_count) << " \n";//added

/***************output statistics******************/
  lteHelper->EnableTraces ();
  
 // Install FlowMonitor on all nodes
  FlowMonitorHelper flowmon;
  //Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
 Ptr<FlowMonitor> monitor = flowmon.Install(ueNodes);
 Ptr<FlowMonitor> monitor1 = flowmon.Install(remoteHost);
 Ptr<FlowMonitor> monitor2 = flowmon.Install(wifiStaNodes);
 Ptr<FlowMonitor> monitor4 = flowmon.Install(LteStaNodes);
 Ptr<FlowMonitor> monitor3 = flowmon.Install(wifiApNode);

  Simulator::Stop(Seconds(simTime));
 	

  Simulator::Run();

   double average_th=0,average_pu_th=0;
   int count=0,count_pu=0;
  
  monitor->CheckForLostPackets ();
  monitor1->CheckForLostPackets ();
  monitor2->CheckForLostPackets ();
  monitor3->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  std::map<FlowId, FlowMonitor::FlowStats> stats2 = monitor2->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
   {
     if (i->first > 0)
       {
          Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
          if (t.destinationAddress=="1.0.0.2"||t.destinationAddress=="3.0.0.11"){

        if (i->second.timeLastRxPacket.GetSeconds()-i->second.timeFirstRxPacket.GetSeconds()!=0)
        {
         average_th=average_th+i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds()-i->second.timeFirstRxPacket.GetSeconds()) / 1000 / 1000;
        if (flowstat[count_pu+1]>0)
        {
             
           average_pu_th=average_pu_th+i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds()-i->second.timeFirstRxPacket.GetSeconds()) / 1000 / 1000/flowstat[count_pu+1];
       
        }
         count++;
         }
       count_pu++;
        }
    }
}

macFile2 << "Average Throughput: " << (average_th/count)<< "\n";
macFile2 << "Average PU Throughput: " << (average_pu_th/count)<< "\n";
macFile2 << "blocked calls: " <<block<< "\n";
macFile2 << "blocked calls including all: " <<block_all<< "\n";
macFile2 << "unconstrained Blocking fraction: " << (block/voice_count)<< "\n";
 


macFile.close();
} 
