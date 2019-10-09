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
#include <ctime>
#include <chrono>
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
  uint32_t explore_count=1;


  //Total number of bytes received using tracing
  //static uint32_t m_bytesTotal;
  static uint32_t m_totalRx;
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
  // Data structures to hold randomized policies (+ and -)
 
  // Make same entries in + and - in case unconstarined MDP
  // uint16_t policy_data_positive[9][11][11];
  //uint16_t policy_voice_positive[9][11][11];
  //uint16_t policy_voice_dep_lte_positive[9][11][11];
  //uint16_t policy_data_dep_lte_positive[9][11][11];
  //uint16_t policy_data_dep_wifi_positive[9][11][11];
  //uint16_t policy_data_negative[9][11][11];
  //uint16_t policy_voice_negative[9][11][11];
  //uint16_t policy_voice_dep_lte_negative[9][11][11];
  //uint16_t policy_data_dep_lte_negative[9][11][11];
  //uint16_t policy_data_dep_wifi_negative[9][11][11];
  double   state_action_Q_value[9][11][11][6][6];// To store Q values for different states and actions
  uint32_t   state_action_visit[9][11][11][6][6];// Number of visits to different states and actions
  uint16_t feasible_action_counter[9][11][11][6];// To store number of feasible actions in a state
  uint16_t feasible_action_set[9][11][11][6];// To store feasible actions in concatenated form e.g:123 etc
  // double   avg_cost=0; // To store average cost (may not be necessary)
  
 //Start and stop time of different servers
  double startarray_voice=0;
  double endarray_voice=0;
  double startarray_data=0;
  double endarray_data=0;
  //double q=0.5; // Prob. with which + policy is taken up
  //double data_thr=5.3; // Constant data Throughput in LTE
 // double voice_thr=0.0312;// Constant Voice Throughput in LTE 
  //double wifi_per_user_thr= [26.0126 14.5304 9.8844 7.4121 5.8939 4.8736 4.1438 3.5973]
  double report_LTE[10],report_wifi[8];// To store SNR in LTE and WiFi
  double inv_snr_wifi,inv_snr_LTE;
  uint16_t index_snr; // To store the index of user with min SNR

  //To generate random arrival and departure time
  double starttime_voice=0;//start time of voice users
  double endtime_voice= 0;//end time of voice users
  double starttime_data=0;//start time of data users
  double endtime_data= 0;//end time of data users
  
 //voice and data user arrival and service rate
  double onebylambdav=1;
  double onebylambdad=1;
  double onebymuv=30;
  double onebymud=30;
  
// To keep track of complexity   
  double complexity=0;
  uint32_t complexity_iteration=0;
  //std::chrono::system_clock::time_point time_start;

 //To keep track of blocking
  double block=0;//(blocking in non-blocking states)
  double block_all=0;// blocking in all states
  static uint32_t flowcount=0;//to count the number of flow
  uint16_t flowstat[300000];// store flow statistics
  double simTime =1002;//Simulation time
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
  void After_Q_learning(uint16_t,double);
  void Q_learning (uint16_t,double,double);
  void policy_choice_data (double, double);
  void policy_choice_voice (double, double);
  double Throughput ();
//  void Periodic_Throughput();
//  double Threshold_value(uint16_t);
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
//std::cout <<   "shorbonash \n";
  Ptr<Packet> m_currentPacket;
  //WifiMacHeader hdr;
 //std::cout <<   "ami okhane \n";
  m_currentPacket = packet->Copy();
  //m_currentPacket->RemoveHeader (hdr);
  //if (hdr.IsData())
  {
    m_totalRx += m_currentPacket->GetSize ();
   // if (m_currentPacket->GetSize()>500 && m_currentPacket->GetSize()<1500)
       //std::cout << "hocche"<<" " << m_totalRx << "size" << m_currentPacket->GetSize() <<"old time"<<old_time.GetSeconds()<<"\n";
  }
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
// std::cout << "ue id:" << uid << "LTE sinr:"<< sinr <<"\n";
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



//calculation of throughput every 10 sec
/*void Periodic_Throughput()
{
Time time = Simulator::Now ();
  //std::cout <<   "ami ekhane \n";
   double interval=25.0;
  //time_of_stay=time.GetSeconds()-old_time.GetSeconds();
  mbps_periodic=((m_totalRx * 8.0) / 1000000+interval*(iteration_throughput)*mbps_periodic)/(interval*(iteration_throughput+1));//yo
//mbps_periodic = ((m_totalRx * 8.0) / 1000000)/(interval);
 // mbps = ((m_totalRx * 8.0) / 1000000)/(10.0*throughput_iteration);//remembar to remove
  std::cout << time.GetSeconds() << "periodic throughput"<<" " << mbps_periodic << "Rxbytes"<<m_totalRx<<"iteration"<<iteration_throughput<<"\n";
  macFile3 << time.GetSeconds() <<" "<< mbps_periodic<< "\n";
  //std::cout << time.GetSeconds() <<"old time"<< " " << old_time.GetSeconds()<< "\n";
  m_totalRx = 0;
  //Simulator::Schedule (Seconds (2.0), &Throughput);
 
  iteration_throughput++;
 if (simTime-time.GetSeconds()>50.0)
 {
  Simulator::Schedule (Seconds (interval), &Periodic_Throughput);
 }
 
}*/

//calculation of throughput for every event
void Event_driven_throughput()
{
 if (first_event==10)
{
  Time time = Simulator::Now ();
   double interval=time_of_stay;
   total_interval=total_interval+interval;
  std::cout << "event driven interval:" << interval << "\n";
  mbps_event=((m_totalRx * 8.0) / 1000000+(total_interval-interval)*mbps_event)/((total_interval));//yo
  sum_step=sum_step+pow(floor(iteration_throughput*0.01)+2,-0.6);
   std::cout << "step size:" << (sum_step)/(iteration_throughput+1) << "\n";
//mbps_periodic = ((m_totalRx * 8.0) / 1000000)/(interval);
 // mbps = ((m_totalRx * 8.0) / 1000000)/(10.0*throughput_iteration);//remembar to remove
  std::cout << time.GetSeconds() << "Event driven throughput"<<" " << mbps_event << "Rxbytes"<<m_totalRx<<"iteration"<<iteration_throughput<<"\n";
  macFile4 << iteration_throughput <<" "<< mbps_event<< " " << sum_step <<"\n";
  //std::cout << time.GetSeconds() <<"old time"<< " " << old_time.GetSeconds()<< "\n";
  m_totalRx = 0;
  //Simulator::Schedule (Seconds (2.0), &Throughput);
 
  iteration_throughput++;
 //if (simTime-time.GetSeconds()>50.0)
 //{
 // Simulator::Schedule (Seconds (interval), &Periodic_Throughput);
 //}
 }
else
 first_event=10;
}



// Calculation of throughput using trace + include cost also
double Throughput () // in Mbps calculated whenever some events occur
{
  Time time = Simulator::Now ();
  //std::cout <<   "ami ekhane \n";
  double mbps=0;
  time_of_stay=time.GetSeconds()-old_time.GetSeconds();
  //mbps = ((m_totalRx * 8.0) / 1000000)/(time.GetSeconds()-old_time.GetSeconds());//remembar to remove
  
  macFile2 << time.GetSeconds() <<"old time"<< " " << old_time.GetSeconds()<< "\n";
  std::cout << time.GetSeconds() <<"old time"<< " " << old_time.GetSeconds()<< "\n";
  //m_totalRx = 0;
  //Simulator::Schedule (Seconds (2.0), &Throughput);
  old_time=Simulator::Now();
  
 //cost involved for voice user blocking
 // if (index_old==1 && action_old==1)
  {
  // std::cout <<   "penalty niyechi \n";
  // std::cout << "penality niyechi: " << (mbps-beta*1) << "\n";
   //return (mbps-beta*1);
   }
  //else
  { 
   //     return mbps;
  }
double wifi_thr[9];
  wifi_thr[0]=0;
 wifi_thr[1]=26.012 ;
 wifi_thr[2]=14.530 ;
 wifi_thr[3]= 9.884 ;
 wifi_thr[4]= 7.412 ;
 wifi_thr[5]= 5.893 ;
 wifi_thr[6]= 4.873 ;
 wifi_thr[7]= 4.143 ;
 wifi_thr[8]= 3.597;
if (index_old==1 && action_old==1)
   mbps=(voiceuser_old*0.03+MIN(datauser_lte_old*5,50)+datauser_wifi_old*wifi_thr[datauser_wifi_old]-beta*1);
if (index_old==1 && action_old==2)
   mbps=((voiceuser_old+1)*0.03+MIN(datauser_lte_old*5,50)+datauser_wifi_old*wifi_thr[datauser_wifi_old]);
if (index_old==1 && action_old==4)
   mbps=((voiceuser_old+1)*0.03+MIN((datauser_lte_old-1)*5,50)+(datauser_wifi_old+1)*wifi_thr[datauser_wifi_old+1]);
if (index_old==2 && action_old==2)
   mbps=(voiceuser_old*0.03+MIN((datauser_lte_old+1)*5,50)+datauser_wifi_old*wifi_thr[datauser_wifi_old]);
if (index_old==2 && action_old==3)
   mbps=(voiceuser_old*0.03+MIN(datauser_lte_old*5,50)+(datauser_wifi_old+1)*wifi_thr[datauser_wifi_old+1]);
if (index_old==3 && action_old==1)
   mbps=((voiceuser_old)*0.03+MIN(datauser_lte_old*5,50)+datauser_wifi_old*wifi_thr[datauser_wifi_old]);
if (index_old==3 && action_old==5)
   mbps=((voiceuser_old)*0.03+MIN((datauser_lte_old+1)*5,50)+(datauser_wifi_old-1)*wifi_thr[datauser_wifi_old-1]);
if (index_old==4 && action_old==1)
   mbps=(voiceuser_old*0.03+MIN((datauser_lte_old)*5,50)+(datauser_wifi_old)*wifi_thr[datauser_wifi_old]);
if (index_old==4 && action_old==5)
   mbps=(voiceuser_old*0.03+MIN((datauser_lte_old+1)*5,50)+(datauser_wifi_old-1)*wifi_thr[datauser_wifi_old-1]);
if (index_old==5 && action_old==1)
   mbps=(voiceuser_old*0.03+MIN(datauser_lte_old*5,50)+(datauser_wifi_old)*wifi_thr[datauser_wifi_old]);
if (index_old==5 && action_old==5)
   mbps=(voiceuser_old*0.03+MIN((datauser_lte_old-1)*5,50)+(datauser_wifi_old+1)*wifi_thr[datauser_wifi_old+1]);
macFile2 << time.GetSeconds() << "reward"<<" " << mbps << "\n";
return mbps;
}

//callback function for voice departure
void MyCallbackDep_voice (uint16_t handle) 
{
    macFile2 << "voice departure" << std::endl;
    macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
    double p,t,reward;
    uint32_t s=0;
    uint16_t cardinality_action=0,concat_action=0,action=0;
  //releasing resource after departure
     if (busy_lte[handle]==1)
     {
        busy_lte[handle]=0;
        voiceuser--;
     }
     reward=Throughput();
    //increase of Q value for last state visited
      After_Q_learning(index_old,reward); 
      
        p = drand48() ;
        
        // Increase state-action pair visit counter
        for (uint16_t i=0;i<6;i++)
        {
                s=s+state_action_visit[datauser_wifi][voiceuser][datauser_lte][3][i];
        }
        
         //how many feasible actions?
         cardinality_action=feasible_action_counter[datauser_wifi][voiceuser][datauser_lte][3];
         
         /// If exploration then choose action at random
         explore_count++;

        if (p<pow(floor(explore_count*0.002)+2,-1))
//        if (p<pow(log(s+3),-1))
        {
                t=cardinality_action*drand48() ;
                macFile2 << "ogo ttttt: " << t << "\n";
                t=ceil(t);
                macFile2<< "ogo ttttt: " << t << "\n";
                concat_action=feasible_action_set[datauser_wifi][voiceuser][datauser_lte][3];
               
                macFile2 << "exploration concat action: " << concat_action << "\n";
              //Take first second or third action based on the value of t
                 
                       while (t>0)
                       {
                        action=concat_action % 10;
                        concat_action=concat_action / 10;
                        t--;
                        action_old=action;
                       }
               macFile2 << "explore action1: " << action << "\n";
  
        }
        //else choose the action with the highest value
        else
        {
              t=-1000;
              macFile2 <<   "exploitation \n";
              concat_action=feasible_action_set[datauser_wifi][voiceuser][datauser_lte][3];
              macFile2 << "concat action: " << concat_action << "\n";
             
              macFile2 << "cardinality action1: " << cardinality_action << "\n";
                for (uint16_t i=0;i<cardinality_action;i++)
                {
                        uint16_t action_temp=concat_action % 10;
                        concat_action=concat_action / 10;
                        macFile2 <<   "ami baire \n";
                        macFile2 << "mago action value: " << state_action_Q_value[datauser_wifi][voiceuser][datauser_lte][3][action_temp] << "\n";
                        if(t<=state_action_Q_value[datauser_wifi][voiceuser][datauser_lte][3][action_temp])
                        {
                                t= state_action_Q_value[datauser_wifi][voiceuser][datauser_lte][3][action_temp];
                                action=action_temp;
                                 action_old=action;
                            macFile2 <<   "ami bhetore \n";
                        }
                }
        macFile2 << "exploit action1: " << action << "\n";
        }
		Event_driven_throughput();//added
        //increase the state action pair visit counter
        state_action_visit[datauser_wifi][voiceuser][datauser_lte][3][action]++;
        //observe reward in terms of throughput
        
        // if  (k==0 && (j~=0||i~=0))
          //    reward=(data_thr*(j)+voice_thr*(i));//PC 24 aug
         //else if k==0 && j==0 && i==0
          //    reward=0;
         //else if k>0
          //    reward=(data_thr*j+wifi_per_user_thr(k)*(k)+voice_thr*(i));//PC 24 Aug
         //else 
     /* if (drand48()<q) // choose positive and negative actions with 0.5 probability
     { action=policy_voice_dep_lte_positive[datauser_wifi][voiceuser][datauser_lte];
     //std::cout << "positive" << std::endl;
      }
     else
    {action=policy_voice_dep_lte_negative[datauser_wifi][voiceuser][datauser_lte];
     //std::cout << "negative" << std::endl;
    }*/
   
   voiceuser_old=voiceuser;
   datauser_lte_old=datauser_lte;
   datauser_wifi_old=datauser_wifi;
   index_old=3;

   //if action=5, offload one data user from wifi to LTE
  if (action==5)
  {
    macFile2 << " action=5" <<  " \n";//added
    found_data_stop=0;//find the running server ansd stop it
    inv_snr_wifi=0;// find UE with min SNR or max inv_SNR
    for (uint16_t u=0;u<8;u++)
    {
      if (busy_data_wifi[u]==1 && found_data_stop==0 && 1/report_wifi[u]>inv_snr_wifi)
      { 
        inv_snr_wifi=1/report_wifi[u];
         std::cout << "SNR wifi1:"<< report_wifi[u]<<"u"<< u<< "\n";
        index_snr=u;
        }
      }
        std::cout << "chosen SNR:"<< 1/inv_snr_wifi<<"\n";
        //u=index_snr;// Index of the UE with min SNR
        found_data_stop=1;
        datauser_wifi--;
        macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
        busy_data_wifi[index_snr]=0;
        Ptr<MyApp> app = wifiStaNodes.Get (index_snr)->GetApplication(0)->GetObject<MyApp> ();
        app->StopApplication ();
        macFile2 << " forcibly starting lte data user @action=5" <<  " \n";//added
//flowcount++;
//flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);
        macFile2 << " voiceuser=: " << voiceuser <<" data_lte=: " << datauser_lte <<" data_wifi= " << datauser_wifi << " \n";//added
        Simulator::Schedule(Seconds(0.0), MyCallbackArrival_data_lte,0.0,(-onebymud*log(drand48())));//stops with parameter 1/mu_d
       }
     } 
 

//callback function for data departure from lte
void MyCallbackDep_data_lte (uint16_t handle) 
{
    macFile2 << "data departure from lte" << std::endl;
    macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
    double p,t,reward;
    uint32_t s=0;
    uint16_t cardinality_action=0,concat_action=0,action=0;
  
 if (busy_lte[handle]==2)
  {
      busy_lte[handle]=0;
      datauser_lte--;
 // }
  
       reward=Throughput();
       After_Q_learning(index_old,reward); 
      
        p = drand48() ;
        // Increase state-action pair visit counter
        for (uint16_t i=0;i<6;i++)
        {
                s=s+state_action_visit[datauser_wifi][voiceuser][datauser_lte][4][i];
        }
        //how many feasible actions?
        cardinality_action=feasible_action_counter[datauser_wifi][voiceuser][datauser_lte][4];
        explore_count++;
        if (p<pow(floor(explore_count*0.002)+2,-1))
        // If exploration then choose action at random
       // if (p<pow(log(s+3),-1))
        {
                t=cardinality_action*drand48() ;
macFile2<< "ogo ttttt: " << t << "\n";
                t=ceil(t);
macFile2 << "ogo ttttt: " << t << "\n";
                concat_action=feasible_action_set[datauser_wifi][voiceuser][datauser_lte][4];
               std::cout << "explore" << std::endl;
                macFile2 << "exploration concat action: " << concat_action << "\n";
              //Take first second or third action based on the value of t
                 
                       while (t>0)
                       {
                        action=concat_action % 10;
                        concat_action=concat_action / 10;
                        t--;
                        action_old=action;
                       }
               
 macFile2 << "explore action2: " << action << "\n";
        }
        //else choose the action with the highest value
        else
        {
                t=-1000;
                macFile2 <<   "exploitation \n";
               std::cout << "exploit" << std::endl;
                concat_action=feasible_action_set[datauser_wifi][voiceuser][datauser_lte][4];
               macFile2 << "concat action: " << concat_action << "\n";
                
                

                for (uint16_t i=0;i<cardinality_action;i++)
                {
                        uint16_t action_temp=concat_action % 10;
                        concat_action=concat_action / 10;
                        if(t<=state_action_Q_value[datauser_wifi][voiceuser][datauser_lte][4][action_temp])
                        {
                                t= state_action_Q_value[datauser_wifi][voiceuser][datauser_lte][4][action_temp];
                                action=action_temp;
                                 action_old=action;
                        }
                }
   macFile2 << "exploit action2: " << action << "\n";
        }
		Event_driven_throughput();//added
        //increase the state action pair visit counter
        state_action_visit[datauser_wifi][voiceuser][datauser_lte][4][action]++;
        //observe reward in terms of throughput
        
        // if  (k==0 && (j~=0||i~=0))
          //    reward=(data_thr*(j)+voice_thr*(i));//PC 24 aug
         //else if k==0 && j==0 && i==0
          //    reward=0;
         //else if k>0
          //    reward=(data_thr*j+wifi_per_user_thr(k)*(k)+voice_thr*(i));//PC 24 Aug
         //else 
 /* if (drand48()<q) // choose positive and negative actions with 0.5 probability
     { action=policy_data_dep_lte_positive[datauser_wifi][voiceuser][datauser_lte];
       std::cout << "positive" << std::endl;}
  else
      {action=policy_data_dep_lte_negative[datauser_wifi][voiceuser][datauser_lte];
       std::cout << "negative" << std::endl;}*/

   voiceuser_old=voiceuser;
   datauser_lte_old=datauser_lte;
   datauser_wifi_old=datauser_wifi;
   index_old=4;
   //if action=5, offload one data user from wifi to LTE
  if (action==5)
  {
    macFile2 << " action=5" <<  " \n";//added
    found_data_stop=0;//find the running server and stop it
    inv_snr_wifi=0;// find UE with min SNR or max inv_SNR
    for (uint16_t u=0;u<8;u++)
    {
      if (busy_data_wifi[u]==1 && found_data_stop==0 && 1/report_wifi[u]>inv_snr_wifi)
      { 
         inv_snr_wifi=1/report_wifi[u];
         std::cout << "SNR wifi1:"<< report_wifi[u]<<"u"<< u<< "\n";
        index_snr=u;
        }
      }
        std::cout << "chosen SNR:"<< 1/inv_snr_wifi<<"\n";
        //u=index_snr;// Index of the UE with min SNR
        found_data_stop=1;
        datauser_wifi--;
        macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
        busy_data_wifi[index_snr]=0;
        Ptr<MyApp> app = wifiStaNodes.Get (index_snr)->GetApplication(0)->GetObject<MyApp> ();
        app->StopApplication ();
        macFile2 << " forcibly starting lte data user @action=5" <<  " \n";//added
//flowcount++;
//flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);
        macFile2 << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << " \n";//added
        Simulator::Schedule(Seconds(0.0), MyCallbackArrival_data_lte,0.0,(-onebymud*log(drand48())));//1/mu_d;
       }
     } 
   }

//callback function for data departure from wifi
void MyCallbackDep_data_wifi (uint16_t handle) 
{
    macFile2 << "data departure from wifi" << std::endl;
    macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
        double p,t,reward;
        uint32_t s=0;
        uint16_t cardinality_action=0,concat_action=0,action=0;
        if (busy_data_wifi[handle]==1)
        {
                busy_data_wifi[handle]=0;
                datauser_wifi--;
       // }
        
     reward=Throughput();
    //increase of Q value for last state visited
      After_Q_learning(index_old,reward); 
        p = drand48() ;
        // Increase state-action pair visit counter
        for (uint16_t i=0;i<6;i++)
        {
                s=s+state_action_visit[datauser_wifi][voiceuser][datauser_lte][5][i];
        }
        cardinality_action=feasible_action_counter[datauser_wifi][voiceuser][datauser_lte][5];

         explore_count++;
        if (p<pow(floor(explore_count*0.002)+2,-1))
        // If exploration then choose action at random
        //if (p<pow(log(s+3),-1))
        {
                t=cardinality_action*drand48() ;
macFile2 << "ogo ttttt: " << t << "\n";
                t=ceil(t);
macFile2 << "ogo ttttt: " << t << "\n";
                concat_action=feasible_action_set[datauser_wifi][voiceuser][datauser_lte][5];
            
               macFile2 << "exploration concat action: " << concat_action << "\n";
               std::cout << "exploration" << std::endl;
                   //Take first second or third action based on the value of t
                       while (t>0)
                       {
                        action=concat_action % 10;
                        concat_action=concat_action / 10;
                        t--;
                        action_old=action;
                       }
             macFile2 << "explore action3: " << action << "\n";
  
        }
        //else choose the action with the highest value
        else
        {
                t=-1000;
                macFile2 <<   "exploitation \n";
                std::cout << "exploit" << std::endl;
                concat_action=feasible_action_set[datauser_wifi][voiceuser][datauser_lte][5];
              macFile2 << "concat action: " << concat_action << "\n";
                
              


                for (uint16_t i=0;i<cardinality_action;i++)
                {
                        uint16_t action_temp=concat_action % 10;
                        concat_action=concat_action / 10;
                        if(t<=state_action_Q_value[datauser_wifi][voiceuser][datauser_lte][5][action_temp])
                        {
                                t= state_action_Q_value[datauser_wifi][voiceuser][datauser_lte][5][action_temp];
                                action=action_temp;
                                 action_old=action;
                        }
                }
  macFile2 << "exploit action3: " << action << "\n";
        }
		Event_driven_throughput();//added
  //increase the state action pair visit counter
        state_action_visit[datauser_wifi][voiceuser][datauser_lte][5][action]++;
        //observe reward in terms of throughput
        
        // if  (k==0 && (j~=0||i~=0))
          //    reward=(data_thr*(j)+voice_thr*(i));//PC 24 aug
         //else if k==0 && j==0 && i==0
          //    reward=0;
         //else if k>0
          //    reward=(data_thr*j+wifi_per_user_thr(k)*(k)+voice_thr*(i));//PC 24 Aug
         //else 
  /*if (drand48()<q) // choose positive and negative actions with 0.5 probability
     {action=policy_data_dep_wifi_positive[datauser_wifi][voiceuser][datauser_lte];
      //std::cout << "positive" << std::endl;
     }
  else
     {action=policy_data_dep_wifi_negative[datauser_wifi][voiceuser][datauser_lte];
     // std::cout << "negative" << std::endl;
    }*/
   voiceuser_old=voiceuser;
   datauser_lte_old=datauser_lte;
   datauser_wifi_old=datauser_wifi;
   index_old=5;

   //if action=5, offload one data user from LTE to wifi
   if (action==5)
  {
    macFile2 << " action=5" <<  " \n";//added
    found_data_stop=0;
     inv_snr_LTE=0;// find UE with min SNR or max inv_SNR
    for (uint16_t u=0;u<10;u++)
    {
      if (busy_lte[u]==2 && found_data_stop==0 && 1/report_LTE[u]>inv_snr_LTE)
      { 
         inv_snr_LTE=1/report_LTE[u];
         std::cout << "SNR lte:"<< report_LTE[u]<<"\n";
         index_snr=u;
       }
     }
        //u=index_snr;// Index of the UE with min SNR
        std::cout << "chosen SNR:"<< 1/inv_snr_LTE<<"\n";
        found_data_stop=1;
        datauser_lte--;
        macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
        busy_lte[index_snr]=0;
        Ptr<MyApp> app =  LteStaNodes.Get (index_snr)->GetApplication(0)->GetObject<MyApp> ();
        app->StopApplication ();
        macFile2 << " forcibly starting wifi data user @action=5" <<  " \n";//added
//flowcount++;
//flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);
       macFile2 << " voiceuser=: " << voiceuser <<" data_lte=: " << datauser_lte <<" data_wifi= " << datauser_wifi << " \n";//added
        Simulator::Schedule(Seconds(0.0), MyCallbackArrival_data_wifi,0.0,(-onebymud*log(drand48())));//mu_d=1;
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
        app->Setup (ns3UdpSocket, sinkAddress, 600,  DataRate ("20Mbps"));// bottleneck is somewhere else
        LteStaNodes.Get (u)->AddApplication (app);
        sourceApps_data_lte[u].Add(app);
        app->SetStartTime (Seconds (0.0));
        app->SetStopTime (Seconds(etime-stime));//mod
        dlPort = dlPort + 1;
        busy_lte[u]=2;
        flowcount++;
//flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);
        flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);//how many users related to this flow
        macFile2<< " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << " \n";//added
        Simulator::Schedule(Seconds(etime-stime), MyCallbackDep_data_lte,u);
   }
  
}
}

//callback function for data arrival in wifi
void MyCallbackArrival_data_wifi (double stime, double etime) 
{
 macFile2 << "data arrival in wifi" << std::endl;
  macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
  uint16_t found_wifi=0;
  for (uint16_t u=0;u<8;u++)
  {
   if (busy_data_wifi[u]==0 && found_wifi==0)
    {
        found_wifi=1;
        datauser_wifi++;
        Ptr<Socket> ns3UdpSocket = Socket::CreateSocket (wifiStaNodes.Get (u),UdpSocketFactory::GetTypeId ());
        Address sinkAddress (InetSocketAddress(apInterfaces.GetAddress (0), port));//Interface betn STA and AP
        PacketSinkHelper sink ("ns3::UdpSocketFactory", sinkAddress);
        sinkApps_data_wifi[u] = sink.Install (wifiApNode.Get (0));
        sinkApps_data_wifi[u].Start (Seconds (0.0));
        sinkApps_data_wifi[u].Stop (Seconds(etime-stime));
        Ptr<MyApp> app = CreateObject<MyApp> ();
        app->Setup (ns3UdpSocket, sinkAddress, 1500,  DataRate ("54Mbps"));
        wifiStaNodes.Get (u)->AddApplication (app);
        sourceApps_data_wifi[u].Add(app);
        app->SetStartTime (Seconds (0.0));
        app->SetStopTime (Seconds(etime-stime));
        port = port + 1;
        busy_data_wifi[u]=1;
        flowcount++;
macFile2 << " flowcount=: " << flowcount << " \n";//added
//flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);
        flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);
       macFile2 << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << " \n";//added
        Simulator::Schedule(Seconds(etime-stime), MyCallbackDep_data_wifi,u);
   }
}
  
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
  if (action==4)
  {
    macFile2 << " action=4" <<  " \n";//added
    for (uint16_t u=0;u<10;u++)
    {
      if (busy_lte[u]==2 && found_data_stop==0 && 1/report_LTE[u]>inv_snr_LTE)
      { 
         inv_snr_LTE=1/report_LTE[u];
         std::cout << "SNR lte:"<< report_LTE[u]<<"\n";
         index_snr=u;
       }
     }
        std::cout << "chosen SNR:"<< 1/inv_snr_LTE<<"\n";
        found_data_stop=1;
        datauser_lte--;
        macFile2 << " Time: " << (Simulator::Now()/1e9) << " \n";//added
        busy_lte[index_snr]=0;
        Ptr<MyApp> app =  LteStaNodes.Get (index_snr)->GetApplication(0)->GetObject<MyApp> ();
        app->StopApplication ();
        macFile2 << " forcibly starting wifi data user @action=4" <<  " \n";//added
        macFile2 << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << " \n";//added
//flowcount++;
//flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);
        Simulator::Schedule(Seconds(0.0), MyCallbackArrival_data_wifi,0.0,(-onebymud*log(drand48())));//mu_d=1;
       }
    
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
//flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);
        flowstat[flowcount]=(voiceuser+datauser_lte+datauser_wifi);
        macFile2 << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << " \n";//added
        //schedule a departure callback function
        Simulator::Schedule(Seconds(etime-stime), MyCallbackDep_voice,u);
     }
  }

 
}

void policy_choice_data(double stime, double etime)
{
/*  uint16_t rat;
  if (drand48()<q) // choose positive and negative actions with 0.5 probability
     {rat=policy_data_positive[datauser_wifi][voiceuser][datauser_lte];
      //std::cout << "positive" << std::endl;
     }
  else
     {rat=policy_data_negative[datauser_wifi][voiceuser][datauser_lte];
      //std::cout << "negative" << std::endl;
     }
  if (rat==2)
     {
        Simulator::Schedule(Seconds(0.0), MyCallbackArrival_data_lte,stime,etime);
        //std::cout << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << "action="<< rat <<" \n";//added
     }
  else if (rat==3)
   {
      Simulator::Schedule(Seconds(0.0), MyCallbackArrival_data_wifi,stime,etime);
     //std::cout << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << "action="<< rat <<" \n";//added
   }
  else 
       std::cout << "data user is blocked" << std::endl;
*/
double reward;
macFile2 << " flowcount=: " <<flowcount<< " \n";//added

 if (first_time==0)
  {
        first_time=10;  
        voiceuser_old=voiceuser;
        datauser_lte_old=datauser_lte;
        datauser_wifi_old=datauser_wifi;
        index_old=2;
        std::cout << "policy choice data" << std::endl;
        old_time=Simulator::Now ();;//update old_time for the first time
        Q_learning(2, stime, etime);
  }
else
{
        reward=Throughput();
        auto time_start = std::chrono::system_clock::now();// Calcualate the time at the start of loop
        After_Q_learning(index_old,reward);
        voiceuser_old=voiceuser;
        datauser_lte_old=datauser_lte;
        datauser_wifi_old=datauser_wifi;
        index_old=2;
        std::cout << "policy choice data1" << std::endl;
        Q_learning(2, stime, etime);
// code complexity
        auto time_end=std::chrono::system_clock::now();
        // std::cout << "time end:" << time_end<<" \n";
        //std::cout << "time end:" << std::chrono::duration_cast<std::chrono::nanoseconds>time_end.count()<<" \n";
        //time_iteration=time_end.GetFemtoSeconds()-time_start.GetFemtoSeconds();// time for a single iteration
        complexity=(std::chrono::duration_cast<std::chrono::microseconds>(time_end-time_start).count()+complexity*complexity_iteration)/(complexity_iteration+1);
        std::cout << "time for a single iteration:" << complexity<<" \n";
       complexity_iteration++;

}
}

void policy_choice_voice(double stime, double etime)
{
   /*uint16_t rat;
   if (drand48()<q) // choose positive and negative actions with 0.5 probability
       {rat=policy_voice_positive[datauser_wifi][voiceuser][datauser_lte];
       
      }
   else
       {rat=policy_voice_negative[datauser_wifi][voiceuser][datauser_lte];
       
      }
  if (rat==2)
  {   //std::cout << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << "action="<< rat <<" \n";//added
      Simulator::Schedule(Seconds(0.0), MyCallbackArrival_voice,stime,etime,2);
  }
  else if (rat==4)
  { 
      Simulator::Schedule(Seconds(0.0), MyCallbackArrival_voice,stime,etime,4);
  }
  else 
       {
               std::cout << "voice user is blocked" << std::endl;
               block_all++;
               if ((voiceuser+datauser_lte)!=10)
               {
                block++;//Count no. of blocked voice calls except states where only possible action is blocking
                std::cout << "blocked calls: " <<block<< "\n";
          
              }
      }*/
double reward;
macFile2 << " flowcount=: " <<flowcount<< " \n";//added
if (first_time==0)
  {
        first_time=10;  
        voiceuser_old=voiceuser;
        datauser_lte_old=datauser_lte;
        datauser_wifi_old=datauser_wifi;
        index_old=1;
        std::cout << "policy choice voice" << std::endl;
        old_time=Simulator::Now ();//update old_time for the first time
        Q_learning(1, stime, etime);
  }
else
{
        reward=Throughput();
        auto time_start = std::chrono::system_clock::now();// Calcualate the time at the start of loop
        After_Q_learning(index_old,reward);
        voiceuser_old=voiceuser;
        datauser_lte_old=datauser_lte;
        datauser_wifi_old=datauser_wifi;
        index_old=1;
        std::cout << "policy choice voice1" << std::endl;
        Q_learning(1, stime, etime);
// code complexity
        auto time_end=std::chrono::system_clock::now();
        // std::cout << "time end:" << time_end<<" \n";
        //std::cout << "time end:" << std::chrono::duration_cast<std::chrono::nanoseconds>time_end.count()<<" \n";
        //time_iteration=time_end.GetFemtoSeconds()-time_start.GetFemtoSeconds();// time for a single iteration
        complexity=(std::chrono::duration_cast<std::chrono::microseconds>(time_end-time_start).count()+complexity*complexity_iteration)/(complexity_iteration+1);
        std::cout << "time for a single iteration:" << complexity<<" \n";
       complexity_iteration++;

}
}

// Function for Q value iteration for voice arrival
void Q_learning (uint16_t index,double stime, double etime)
{
        
        double p,s=0,t;//reward=0;
        uint16_t cardinality_action=0,concat_action=0,action=0;
        p = drand48() ;
        // Increase state-action pair visit counter
        for (uint16_t i=0;i<6;i++)
        {
                s=s+state_action_visit[datauser_wifi][voiceuser][datauser_lte][index][i];
                macFile2 << "state_action visit: " <<s<< "\n";
        }
        cardinality_action=feasible_action_counter[datauser_wifi][voiceuser][datauser_lte][index];
        macFile2 << "no of feasible actions in this state: " << cardinality_action << "\n";

        // If exploration then choose action at random
        explore_count++;
        if (p<pow(floor(explore_count*0.002)+2,-1))
 //       if (p<pow(log(s+3),-1))
        {
                t=cardinality_action*drand48() ;
macFile2 << "ogo ttttt: " << t << "\n";
                t=ceil(t);
macFile2 << "ogo tttt: " << t << "\n";
               macFile2 << " voice=: " << voiceuser <<" data_lte=: " << datauser_lte <<" data_wifi= " << datauser_wifi << "index="<< index <<" \n";//added
                concat_action=feasible_action_set[datauser_wifi][voiceuser][datauser_lte][index];
                macFile2 << "concat action: " << concat_action << "\n";
                macFile2 <<   "exploration \n";
                std::cout << "exploration" << std::endl;
               // if (t<1)
                       while (t>0)
                       {
                        action=concat_action % 10;
                        concat_action=concat_action / 10;
                        t--;
                        action_old=action;
                       }
                macFile2 << "action taken: " << action << "\n";
               // else if (1<=t<=2)
                //        action=2;
                //else
                //        action=4;
  
        }
        //else choose the action with the highest value
        else
        {
                t=-1000;
               macFile2 <<   "exploitation \n";
               std::cout << "exploit" << std::endl;
                concat_action=feasible_action_set[datauser_wifi][voiceuser][datauser_lte][index];
               macFile2 << "concat action: " << concat_action << "\n";
                for (uint16_t i=0;i<cardinality_action;i++)
                {
                        uint16_t action_temp=concat_action % 10;
                        concat_action=concat_action / 10;
                        if(t<=state_action_Q_value[datauser_wifi][voiceuser][datauser_lte][index][action_temp])
                        {
                                t= state_action_Q_value[datauser_wifi][voiceuser][datauser_lte][index][action_temp];
                                action=action_temp;
                                action_old=action;
                        }
                }
               macFile2 << "exploit action4: " << action << "\n";
        }
        Event_driven_throughput();//added
        state_action_visit[datauser_wifi][voiceuser][datauser_lte][index][action]++;
        //observe reward in terms of throughput
        //reward=Throughput();
        // if  (k==0 && (j~=0||i~=0))
          //    reward=(data_thr*(j)+voice_thr*(i));//PC 24 aug
         //else if k==0 && j==0 && i==0
          //    reward=0;
         //else if k>0
          //    reward=(data_thr*j+wifi_per_user_thr(k)*(k)+voice_thr*(i));//PC 24 Aug
         //else
if (index==1)
{
 if (action==2)
  {   
      Simulator::Schedule(Seconds(0.0), MyCallbackArrival_voice,stime,etime,2);
  }
  else if (action==4)
  { 
      Simulator::Schedule(Seconds(0.0), MyCallbackArrival_voice,stime,etime,4);
  }
  else 
      {
               macFile2 << "voice user is blocked" << std::endl;
               block_all++;
               if ((voiceuser+datauser_lte)!=10)
               {
                block++;//Count no. of blocked voice calls except states where only possible action is blocking
               macFile2 << "blocked calls: " <<block<< "\n";
          
              }
      }
}
else if (index==2)
{
if (action==2)
     {
        Simulator::Schedule(Seconds(0.0), MyCallbackArrival_data_lte,stime,etime);
        //std::cout << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << "action="<< rat <<" \n";//added
     }
  else if (action==3)
   {
      Simulator::Schedule(Seconds(0.0), MyCallbackArrival_data_wifi,stime,etime);
     //std::cout << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << "action="<< rat <<" \n";//added
   }
  else 
       std::cout << "data user is blocked" << std::endl;
}
else //if (index==3)||(index==4)||(index=5)
{
 //do nothing
}
//After_Q_learning(index);
//else
//{
// time_start = std::chrono::system_clock::now();// Calcualate the time at the start of loop
//}
}
/*
// Function for Q value iteration for data arrival
Q_learning_data_arrival(int x, int y, int z)
{
        double p,s=0,t;
        p = drand48() ;
        for (i=0;i<5;i++)
        {
                s=s+state_action_visit[x][y][z][2][i];
        }
        if (p<pow(log(s+3),-1))
        {
                t=2*drand48() ;
                if (t<1)
                        action=2;
                //else if (1<=t<=2)
                 //       action=2;
                else
                        action=3;
  
        }
        else
        {
                t=0;
                for (i=0;i<5;i++)
                {
                        if(t<state_action_Q_value[x][y][z][2][i])
                        {
                                t= state_action_Q_value[x][y][z][2][i];
                                action=i;
                        }
                }
        }
        state_action_visit[x][y][z][2][action]++;
        reward= Throughput();
if (action==2)
     {
        Simulator::Schedule(Seconds(0.0), MyCallbackArrival_data_lte,stime,etime);
        //std::cout << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << "action="<< rat <<" \n";//added
     }
  else if (action==3)
   {
      Simulator::Schedule(Seconds(0.0), MyCallbackArrival_data_wifi,stime,etime);
     //std::cout << " i=: " << voiceuser <<" j=: " << datauser_lte <<" k= " << datauser_wifi << "action="<< rat <<" \n";//added
   }
  else 
       std::cout << "data user is blocked" << std::endl;
}
*/
// Function for Q value iteration for voice/data departure
/*Q_learning_voice_data_departure(int x, int y, int z, int index)
{
        double p,s=0,t;
        p = drand48() ;
        for (i=0;i<5;i++)
        {
                s=s+state_action_visit[x][y][z][index][i];
        }
        if (p<pow(log(s+3),-1))
        {
                t=2*drand48() ;
                if (t<1)
                        action=1;
                //else if (1<=t<=2)
                 //       action=2;
                else
                        action=5;
  
        }
        else
        {
                t=0;
                for (i=0;i<5;i++)
                {
                        if(t<state_action_Q_value[x][y][z][index][i])
                        {
                                t= state_action_Q_value[x][y][z[index][i];
                                action=i;
                        }
                }
        }
        state_action_visit[x][y][z][index][action]++;
       reward= Throughput();
}*/

// Updation of Q values after state change for voice arrival
void After_Q_learning(uint16_t index,double reward)
{
 
  double reward1=reward;
  double cost;
 //Q(s,a)=(1-rho)Q(s,a)
// double rho=pow(state_action_visit[voiceuser_old][datauser_lte_old][datauser_wifi_old][index_old][action_old]+3,-0.6);
double rho=pow(floor(iteration_fast*0.01)+2,-0.6);
//fast time sacle
  //double rho=pow(iteration_fast,-0.7);
  macFile2 << "iteration fast:" << iteration_fast<<" \n"; 
  iteration_fast=iteration_fast+1;
  macFile2 << "time of stay:" << time_of_stay<<" \n"; 
//slow time scale
  double rho1=10*pow(iteration_slow,-1);
//double rho1=pow(floor(iteration_slow*0.01)+2,-1);
  macFile2<< "iteration slow:" << iteration_slow<<" \n"; 
  iteration_slow=iteration_slow+1;
 if (index_old==1 && action_old==1)
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
 std::cout << "beta updated:" << beta<<" \n"; 

  state_action_Q_value[datauser_wifi_old][voiceuser_old][datauser_lte_old][index_old][action_old]=(1-rho)*state_action_Q_value[datauser_wifi_old][voiceuser_old][datauser_lte_old][index_old][action_old];
 //Q(s,a)=Q(s,a)+rho(R+max_a1(Q(s1,a1))) 
                double t=-1000;
                for (uint16_t i=0;i<6;i++)
                {
                        if(t < state_action_Q_value[datauser_wifi][voiceuser][datauser_lte][index][i])
                        {
                                t= state_action_Q_value[datauser_wifi][voiceuser][datauser_lte][index][i];
                                //action=i;
                        }
                }
        //time_of_stay has been made equal to 1. Activate this line and deactivate the next line depending on the case

  //state_action_Q_value[datauser_wifi_old][voiceuser_old][datauser_lte_old][index_old][action_old]+=rho*(reward1+t-state_action_Q_value[0][0][0][1][2]*time_of_stay);
   state_action_Q_value[datauser_wifi_old][voiceuser_old][datauser_lte_old][index_old][action_old]+=rho*(reward1+t-state_action_Q_value[0][0][0][1][2]*1);

 macFile2 << "Q updated"<< state_action_Q_value[datauser_wifi_old][voiceuser_old][datauser_lte_old][index_old][action_old] <<  " ei ne \n";
  macFile2 << "voiceold:" <<voiceuser_old <<"datalteold:"<<datauser_lte_old <<"datawifiold:"<< datauser_wifi_old <<"indexold:"<<index_old<<"actionold:"<<action_old<<"ami out \n";  
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
  //Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (2008));
  //Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (9000000));
 // Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (9000000));
  //Config::SetDefault ("ns3::WifiMacQueue::MaxPacketNumber", UintegerValue (400));//hatao
  //Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(MilliSeconds(9000000)));
  //uint16_t option=0;// option=0 : wifi flow active =1: LTE flow active

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

  macFile.open ("beta_1_2_time_beta_revised.txt");
  macFile << "iteration\tbeta\n";
  macFile1.open ("stat_1_2_time_beta_revised.txt");
  macFile1 << "voiceuser\t data_lte \t data_wifi \t index \t opt action \t opt act value \t state action visit\n";
 macFile2.open ("alltrace_1_2_time_beta_revised.txt");
 macFile3.open ("throughput_1_2_time_beta_revised.txt");
macFile4.open ("event_throughput_1_2_time_beta_revised.txt");
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

  uint16_t count1=0;
//Initialize action set and action counter for states
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
                                        //std::cout << "i:" <<i<<"j:"<<j<<"k:"<<k<<"index:"<<index<<"\n";
                                        //std::cout << "count1"<< count1 <<  "ami age \n";
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
     
             if ((index==1)||((index==2)&&((j+k)==10)&&(i==8))||(index==3)||(index==4)||(index==5))
             {
               //std::cout << "i:" <<i<<"j:"<<j<<"k:"<<k<<"index:"<<index<<"\n";
                //if ((i==0) && (j==0) && (k==0) && (index==1))
                 // std::cout << "ohmygod1" << std::endl;
                  feasible_action_set[i][j][k][index]=feasible_action_set[i][j][k][index]*10+1;
                  feasible_action_counter[i][j][k][index]++;
              }
            if (((index==1)||(index==2)) && ((j+k)<10))
            {
             // if ((i==0) && (j==0) && (k==0) && (index==1))
              //std::cout << "ohmygod2" << std::endl;
              //std::cout << "i:" <<i<<"j:"<<j<<"k:"<<k<<"\n";
              feasible_action_set[i][j][k][index]=feasible_action_set[i][j][k][index]*10+2;
              feasible_action_counter[i][j][k][index]++;
           }
           if ((index==2) && (i<8))
           {
             //if ((i==0) && (j==0) && (k==0) && (index==1))
              //std::cout << "ohmygod3" << std::endl;
             //std::cout << "i:" <<i<<"j:"<<j<<"k:"<<k<<"\n";
             feasible_action_set[i][j][k][index]=feasible_action_set[i][j][k][index]*10+3;
              feasible_action_counter[i][j][k][index]++;
            }
             if ((index==1) && ((j+k)<=11) && k>0 && i<8)
             {
               //if ((i==0) && (j==0) && (k==0) && (index==1))
               //std::cout << "ohmygod4" << std::endl;
              //std::cout << "i:" <<i<<"j:"<<j<<"k:"<<k<<"\n";
               feasible_action_set[i][j][k][index]=feasible_action_set[i][j][k][index]*10+4;
               feasible_action_counter[i][j][k][index]++;
             }
            if  (((index==3)||(index==4)) && ((j+k)<11) && (i>0))
            {
             //std::cout << "i:" <<i<<"j:"<<j<<"k:"<<k<<"\n";
             feasible_action_set[i][j][k][index]=feasible_action_set[i][j][k][index]*10+5;
             feasible_action_counter[i][j][k][index]++;
            }
            if ((index==5) && (k>0) && (i<8))
            {
              //std::cout << "i:" <<i<<"j:"<<j<<"k:"<<k<<"\n";
              feasible_action_set[i][j][k][index]=feasible_action_set[i][j][k][index]*10+5;
              feasible_action_counter[i][j][k][index]++;
            }
         count1++;

   }
  }
 }
 // std::cout << "count1"<< count1 <<  "ami ekhane \n";
 }
  //std::cout <<   "ami ekhaneo \n";
  macFile2 << "omg"<< feasible_action_set[0][1][0][1] <<  "ami age \n";
}
                   
// std::cout << "i:" <<i<<"j:"<<j<<"k:"<<k<<"index:"<<index<<"ami out" \n";                   

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

 //for wifi
 //Ipv4StaticRoutingHelper ipv4RoutingHelper1;
 // Ptr<Ipv4StaticRouting> remoteHostStaticRouting1 = ipv4RoutingHelper1.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
//  //remoteHostStaticRouting1->AddNetworkRouteTo (Ipv4Address ("3.0.0.0"), Ipv4Mask ("255.0.0.0"), Ipv4Address("1.0.0.1"),1);
// //Ptr<Ipv4StaticRouting> pgwStaticRouting = ipv4RoutingHelper1.GetStaticRouting (pgw->GetObject<Ipv4> ());
// remoteHostStaticRouting1->AddNetworkRouteTo (Ipv4Address ("3.0.0.0"), Ipv4Mask ("255.0.0.0"), Ipv4Address ("2.0.0.2"), 1);
// Ptr<Ipv4StaticRouting> apStaticRouting = ipv4RoutingHelper1.GetStaticRouting (wifiap->GetObject<Ipv4> ());
// apStaticRouting->SetDefaultRoute (Ipv4Address ("3.0.0.0"), 1);
// //apStaticRouting->AddNetworkRouteTo (Ipv4Address ("3.0.0.0"), Ipv4Mask ("255.0.0.0"),1);

 // NodeContainer ueNodes;
  NodeContainer enbNodes;
  enbNodes.Create(numberOfBS);
  ueNodes.Create(numberOfNodes);

  

  /*************** Create wifi phy, channel and mac for WAPs ************/
  //WIFI PHY layer
  //L=140.7+36.7log_10 R for 2GHz, R in km
 Config::SetDefault( "ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue (30.6));
 Config::SetDefault("ns3::LogDistancePropagationLossModel::Exponent", DoubleValue (3.67));
  //channel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel", "ReferenceLoss", DoubleValue (15.3), "Exponent", DoubleValue (3.67));
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
  //phy.Set("TxGain",DoubleValue(5));
  //phy.Set("EnergyDetectionThreshold", DoubleValue(-76.0));
  phy.SetChannel (channel.Create ());

  // WIFI MAC layer
  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetStandard (WIFI_PHY_STANDARD_80211g); // using 802.11g standard
  //wifi.SetRemoteStationManager ("ns3::AarfWifiManager");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                              "DataMode",StringValue ("ErpOfdmRate54Mbps"), "ControlMode",StringValue ("ErpOfdmRate6Mbps"));
  //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                             // "DataMode",StringValue ("ErpOfdmRate54Mbps"), "ControlMode",StringValue ("ErpOfdmRate54Mbps"));
  NqosWifiMacHelper mac = NqosWifiMacHelper::Default ();

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
  //mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
  //"X", DoubleValue (100.0),
  //"Y", DoubleValue (100.0),
 // "rho", DoubleValue (radius));//uniformly distributed in a disc of radius rho
  mobility.SetPositionAllocator(uePosition);
  mobility.Install(ueNodes);
  mobility.SetPositionAllocator(positionAlloc1);
  mobility.Install(enbNodes);
  mobility.SetPositionAllocator(positionAlloc);
  mobility.Install(remoteHostContainer);

  positionAlloc2->Add (Vector(50.0, 30.0, apAntennaHeight));//position of AP
  // (not within 35 m from eNB)
  // Position of UEs attached to AP1 inside a radius of 30m
    for (uint32_t i = 0; i < wifiStaNodes.GetN(); ++i)
    {
 
      int v1 = rand() % 60+20; 
      int v2 = rand() % 60+0; 
       if (pow(v1-50,2)+pow(v2-30,2)<=pow(30,2))
         WifiStaPosition->Add (Vector(v1, v2, msAntennaHeight));
       else
         i--;

     // std::cout << " sta1 location1: " << v1-50<< "sta1 location2: "<<v2-30<<" \n";//added
     
    } 
  mobility.SetPositionAllocator(WifiStaPosition);

  // Position of UEs attached to eNB 1
  //mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
  //  "X", DoubleValue (50.0),
  //"Y", DoubleValue (20.0),
   // "rho", DoubleValue (10.0));//uniformly distributed in a disc of radius 10 m coz wifi AP range is smaller(not 100 m like LTE BS)

  mobility.Install(wifiStaNodes);
 for (uint32_t i = 0; i < LteStaNodes.GetN(); ++i)
    {
     
      int v1 = rand() % 60+20; 
      int v2 = rand() % 60+0; 
      //std::cout << " location1: " << v1-50 << " location2: "<<v2-30<<" \n";//added
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
  staIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (staLteDevs));//remember
  

  // Assign IP address to UEs, and install applications
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
  for (uint32_t u = 0; u < LteStaNodes.GetN (); ++u)//remember
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
        lteHelper->Attach (staLteDevs.Get(i), enbLteDevs.Get(0));//remember

      
        // side effect: the default EPS bearer will be activated
       
      }




  /************************ Install and start applications on UEs and remote host*************/

//---------------voice policy start
   uint32_t voice_count=0, data_count=0;
   for (uint16_t i = 0; i < 100; i++)
   {
     starttime_data= starttime_data+(-onebylambdad*log(drand48())); //1/lambdad
     starttime_data=0;   //for (uint32_t i = 0; i<5; i=i+1)
    }
   
   while (endtime_data <(simTime-50.0))//remember to remove
    //while (data_count<3)
    {
  
     starttime_data= starttime_data+(-onebylambdad*log(drand48()));//1/lambda_d
     endtime_data= starttime_data+ (-onebymud*log(drand48()));//1/mu_d
     
  
     startarray_data=starttime_data;
     endarray_data=endtime_data;
    // if (data_count==0)
    {
        std::cout << " start Time for data: " << (startarray_data) << " \n";//added
        std::cout << " end Time for data: " << (endarray_data) << " \n";//added
    }
     data_count++; 
     Simulator::Schedule(Seconds(starttime_data),policy_choice_data,startarray_data,endarray_data);
  }

   while (endtime_voice <(simTime-50.0))   //remember to remove
  //while (voice_count<3)
   {
   
     starttime_voice= starttime_voice+(-onebylambdav*log(drand48()));//1/lambda_v
     endtime_voice= starttime_voice+ (-onebymuv*log(drand48()));//1/mu_v
    startarray_voice=starttime_voice;
    endarray_voice=endtime_voice;
    //if (voice_count==0)
    {
        std::cout << " start Time for voice: " << (startarray_voice) << " \n";//added
        std::cout << " end Time for voice: " << (endarray_voice) << " \n";//added
    }
    voice_count++;
    Simulator::Schedule(Seconds(startarray_voice), policy_choice_voice,startarray_voice,endarray_voice);
   }

    
   std::ostringstream oss;
   std::ostringstream oss1;
    oss << "/NodeList/" << wifiApNode.Get (0)->GetId() << "/$ns3::Ipv4L3Protocol/Rx"; // for AP it works as it has only one interface
    oss1 << "/NodeList/" << remoteHost->GetId() << "/$ns3::Ipv4L3Protocol/Rx"; // for AP it works as it has only one interface
Config::Connect ("/NodeList/*/DeviceList/*/$ns3::LteUeNetDevice/LteUePhy/ReportCurrentCellRsrpSinr",
                   MakeCallback (&ReportCurrentCellRsrpSinr));//RSRP reporting in LTE
Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/RxOk",
                   MakeCallback (&PhyRxOkTrace));// RSSI reporting in WiFi

    Config::Connect (oss.str(), MakeCallback (&IPv4ReceivedPackets));
    Config::Connect (oss1.str(), MakeCallback (&IPv4ReceivedPackets));
    //Simulator::Schedule (Seconds (25.0), &Periodic_Throughput);

//Trace function to call
//Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/RxOk", MakeCallback (&PhyRxOkTrace));
//Config::Connect("/NodeList/2/$ns3::Ipv4L3Pro.tocol/Rx" , MakeCallback(&IPv4ReceivedPackets));
//Config::Connect("/NodeList/wifiApNode.Get (0)->GetId()/$ns3::Ipv4L3Protocol/Rx" , MakeCallback(&IPv4ReceivedPackets));
//   Config::Connect("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",MakeCallback (&ReceivePacket));
//-------------voice policy end



//-----------data policy start
// for (uint32_t i = 0; i<5; i=i+1)

//------data policy end
std::cout << " -1end Time for voice: " << (endarray_voice) << " \n";//added
std::cout << " -1end Time for data: " << (endarray_data) << " \n";//added
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
 	
 // AnimationInterface anim ("animation_comb.xml");
//for (uint32_t i = 0; i < LteStaNodes.GetN(); ++i)
//{
//anim.UpdateNodeColor (LteStaNodes.Get (i), 0, 255, 0);
//} 
//lteHelper->EnablePhyTraces ();
//lteHelper->EnableMacTraces ();
//lteHelper->EnableRlcTraces ();
//lteHelper->EnablePdcpTraces ();
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
          //std::cout << "Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
          //std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
          //std::cout << "  Tx MBytes:   " << i->second.txBytes / 1000.0 / 1000.0 << "\n";
         // std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / (i->second.timeLastTxPacket.GetSeconds()-i->second.timeFirstTxPacket.GetSeconds()) / 1000 / 1000  << " Mbps\n";
          //std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
          //std::cout << "  Rx MBytes:   " << i->second.rxBytes / 1000.0 / 1000.0 << "\n";
         // std::cout << "  THROUGHPUT: " << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds()-i->second.timeFirstRxPacket.GetSeconds()) / 1000 / 1000  << " Mbps\n";
        //std::cout << "Time: " << (GetSeconds() i->second.timeLastRxPacket - GetSeconds() i->second.timeFirstRxPacket) / 1000 / 1000 / 1000 << " \n";
          //std::cout << " Time: " << (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstRxPacket.GetSeconds()) << " \n";//added
         //std::cout << " Start Time: " << (i->second.timeFirstRxPacket.GetSeconds() ) << " \n";//added
         //std::cout << " End Time: " << (i->second.timeLastRxPacket.GetSeconds() ) << " \n";//added
         //std::cout << "TX Start Time: " << (i->second.timeFirstTxPacket.GetSeconds() ) << " \n";//added
         //std::cout << "TX End Time: " << (i->second.timeLastTxPacket.GetSeconds() ) << " \n";//added
        if (i->second.timeLastRxPacket.GetSeconds()-i->second.timeFirstRxPacket.GetSeconds()!=0)
        {
         average_th=average_th+i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds()-i->second.timeFirstRxPacket.GetSeconds()) / 1000 / 1000;
        if (flowstat[count_pu+1]>0)
        {
             // std::cout << " FLOWSTAT: " << flowstat[count_pu+1] << " \n";//added
  average_pu_th=average_pu_th+i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds()-i->second.timeFirstRxPacket.GetSeconds()) / 1000 / 1000/flowstat[count_pu+1];
       
        }
         count++;
         }
       count_pu++;
        }
    }
}
//std::cout << "Count: " << count<< "\n";
//std::cout << "Count_pu: " << count_pu<< "\n";
//std::cout << "my app uc copy" <<"\n";
macFile2 << "Average Throughput: " << (average_th/count)<< "\n";
macFile2 << "Average PU Throughput: " << (average_pu_th/count)<< "\n";
macFile2 << "blocked calls: " <<block<< "\n";
macFile2 << "blocked calls including all: " <<block_all<< "\n";
macFile2 << "-1 unconstrained Blocking fraction lambdav=1/80 1000 s: " << (block/voice_count)<< "\n";

double   action_opt,t1;
uint32_t visito=0;
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
                                      t1=-1000;
                                      for (uint16_t action=1;action<6;action++) 
                                      {
                                          if (t1 <=state_action_Q_value[i][j][k][index][action])
                                           {
                                                t1=state_action_Q_value[i][j][k][index][action];
                                                action_opt=action;
                                                visito=state_action_visit[i][j][k][index][action];
                                           }
                                        
                                       }
if (visito>2)//sufficient number of visit
{
macFile1 << j << "\t";
macFile1 << k << "\t";
macFile1 << i << "\t";
macFile1 << index << "\t";
macFile1 << action_opt << "\t";
macFile1 << t1 << "\t";
macFile1 << visito << "\n";
}
//std::cout << "voiceuser:" <<j<<"data_lte:"<<k<<"data_wifi:"<<i<<"index:"<<index<<"opt action:"<<action_opt<<"opt act value:"<<t1<< "state action visit:"<< visito<<"\n";
//std::cout << "state action visit:"<< visito<<"\n";
                                        //std::cout << "count1"<< count1 <<  "ami age \n";
                                }
                      }
              }
       }
 }

for (uint16_t i=1;i<50000;i++)
  {
    macFile << time_array[i] << "\t";
    macFile << beta_array[i] << "\n"; 
    //std::cout << "beta:"<< beta_array[i]<<"\n";
  }
macFile.close();
} 
