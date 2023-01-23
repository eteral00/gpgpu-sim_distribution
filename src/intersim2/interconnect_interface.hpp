// Copyright (c) 2009-2013, Tor M. Aamodt, Dongdong Li, Ali Bakhoda
// The University of British Columbia
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright notice, this
// list of conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.
// Neither the name of The University of British Columbia nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef _INTERCONNECT_INTERFACE_HPP_
#define _INTERCONNECT_INTERFACE_HPP_

#include <vector>
#include <queue>
#include <iostream>
#include <map>

#include <utility>
#include <unordered_map>
#include <set>
#include <string>
#include <inttypes.h>
#include "../gpgpu-sim/mem_fetch.h"
#include "../cuda-sim/half.h"

using namespace std;


// Do not use #include since it will not compile in icnt_wrapper or change the makefile to make it
class Flit;
class GPUTrafficManager;
class IntersimConfig;
class Network;
class Stats;


struct AverageRangePair {
  AverageRangePair() {
    _average = 0;
    _range = 0;
  }

  AverageRangePair(float average, float range) {
    _average = average;
    _range = range;
  }

  ~AverageRangePair() {

  }

  float _average;
  float _range;
};

//TODO: fixed_lat_icnt, add class support? support for signle network

class InterconnectInterface {
public:
  InterconnectInterface();
  virtual ~InterconnectInterface();
  static InterconnectInterface* New(const char* const config_file);
  virtual void CreateInterconnect(unsigned n_shader,  unsigned n_mem);
  
  //node side functions
  virtual void Init();
  virtual void Push(unsigned input_deviceID, unsigned output_deviceID, void* data, unsigned int size);
  virtual void* Pop(unsigned ouput_deviceID, unsigned subnet, long long unsigned curCycle); // Khoa
  // virtual void* Pop(unsigned ouput_deviceID); // Khoa
  virtual void Advance(long long unsigned curCycle); // Khoa
  // virtual void Advance(); // Khoa
  virtual bool Busy() const;
  virtual bool HasBuffer(unsigned deviceID, unsigned int size, unsigned subnet) const;
  // virtual bool HasBuffer(unsigned deviceID, unsigned int size) const;
  virtual void DisplayStats() const;
  virtual void DisplayOverallStats() const;
  unsigned GetFlitSize() const;
  
  virtual void DisplayState(FILE* fp) const;
  
  //booksim side functions
  void WriteOutBuffer( int subnet, int output, Flit* flit );
  void Transfer2BoundaryBuffer(int subnet, int output);
  
  int GetIcntTime() const;
  
  Stats* GetIcntStats(const string & name) const;
  
  Flit* GetEjectedFlit(int subnet, int node);
  


  // Khoa, 2022/07/
  virtual bool HasVCBuffer(unsigned deviceID, unsigned int size, unsigned subnet, unsigned mode = 2) const;
  unsigned getIcntID(unsigned deviceID) {
    return _node_map[deviceID];
  }
  unsigned getDeviceID(unsigned icntID) {
    return _reverse_node_map[icntID];
  }
  unsigned getNumOfShaders() {
    return _n_shader;
  }
  unsigned getNumOfMemNodes() {
    return _n_mem;
  }
  void printLUT();
  void printLUT1();
  void printLUT2();
  // void printLUT1(FILE* resOutFile);
  // void printLUT2(FILE* resOutFile);
  void printMfData(mem_fetch *mf, std::string filePath);
  void changeMfData(mem_fetch *mf, unsigned srcIcntID);

  std::string toBitString(void const * const pointer, unsigned sizeInByte);
  // uint16_t approx_Bit_general(void* value, unsigned wordByteSize, unsigned bitMask);
  uint16_t approx_Bit_16(uint16_t value, uint16_t bitMask);
  uint16_t approx_Bit_16_2Steps(uint16_t value, unsigned expLimit, unsigned numberOfMantisBit);
  void approx_Bit(void* value, unsigned wordByteSize, unsigned bitMask);
  std::string approx_Average(uint16_t * valueBlock, unsigned numberOfVal);
  std::string approx_AverageBinary(uint16_t * valueBlock, unsigned numberOfVal);
  // void approx_halfCeil(half* value);
  AverageRangePair approx_Average_2(uint16_t * valueBlock, unsigned numberOfVal);

  unsigned getHomebase(long long unsigned address);
  unsigned getHomebase(mem_fetch * mf);


  GPUTrafficManager* _traffic_manager;
  vector < vector < vector < unsigned > > > homebaseMap;
  

  unsigned redirectedCount;
  unsigned pushCount;
  unsigned popCount;
  unsigned popCount_noData;
  unsigned readRequestPushCount, writeRequestPushCount, readReplyPushCount, writeReplyPushCount;
  unsigned readRequestPopCount, writeRequestPopCount, readReplyPopCount, writeReplyPopCount;
  unsigned changedCount;

  unsigned sameAddressAtSameClusterAccessCountTracking;
  unsigned sameAddressAtOtherClusterAccessCountTracking;
  unsigned sameAddressAtBothClustersAccessCountTracking;
  unsigned sameAddressAtEitherClusterAccessCountTracking;
  unsigned sameAddressAtEitherClusterAccessCountTracking_100;
  unsigned sameAddressAtEitherClusterAccessCountTracking_200;

  unsigned sameContentAtSameClusterAccessCountTracking;
  unsigned sameContentAtOtherClusterAccessCountTracking;
  unsigned sameContentAtBothClustersAccessCountTracking;
  unsigned sameContentAtEitherClusterAccessCountTracking;
  unsigned sameContentAtEitherClusterAccessCountTracking_100;
  unsigned sameContentAtEitherClusterAccessCountTracking_200;
  
  unsigned approxContentAtSameClusterAccessCountTracking;
  unsigned approxContentAtOtherClusterAccessCountTracking;
  unsigned approxContentAtBothClustersAccessCountTracking;
  unsigned approxContentAtEitherClusterAccessCountTracking;
  unsigned approxContentAtEitherClusterAccessCountTracking_100;
  unsigned approxContentAtEitherClusterAccessCountTracking_200;
  
  std::unordered_map< unsigned, unsigned > redirectedPackets;
  std::vector< unsigned > redirectPacketPerDestCount;
  std::vector< unsigned > receivedPacketPerDestCount;
  std::vector< std::vector< unsigned > > routedPacketPerDestCount;

  std::unordered_map< std::string, std::unordered_map< long long unsigned, std::set< unsigned > > > contentAddressMC;
  std::unordered_map< std::string, std::unordered_map< unsigned, std::set<long long unsigned> > > contentMCAddress;
  std::unordered_map< std::string, std::set< pair <long long unsigned, unsigned> > > contentAddresses;
  std::unordered_map< std::string, std::set< unsigned > > contentMCs;

  // std::unordered_map< std::string, std::vector < std::string > > contentMatches;

  
  bool printNFlit;
  unsigned totalPacketHopsCount;
  unsigned totalHops_ReadRequest;
  unsigned totalHops_ReadReply;
  unsigned totalHops_Update;
  unsigned totalLUTRead_Homebase;
  unsigned totalLUTWrite_Homebase;
  unsigned totalLUTRead_MC;
  unsigned totalLUTWrite_MC;
  long long unsigned totalReadRequestWaitTimeAtMC;
  long long unsigned totalAllRequestWaitTimeAtMC;
  long long unsigned totalReplyWaitTimeAtMC;
  unsigned totalReadRequest;
  unsigned totalWriteRequest;
  unsigned totalReadReply_MC;
  unsigned totalReadReply_peer;

  unsigned wrongSpaceCount;
  
  unsigned numOfDistinctContent;
  unsigned numOfDistinctContent_approx;
  unsigned numOfDistinctContent2;
  unsigned numOfDistinctContent_approx2;

  std::unordered_map< long long unsigned, std::set< long long unsigned > > sameAddress_OverTime;
  std::unordered_map< long long unsigned, long long unsigned > sameAddress_LastAccessTime; //
  std::unordered_map< long long unsigned, std::set< unsigned > > sameAddress_AtMultiClusters;
  // std::unordered_map< long long unsigned, std::set< pair < unsigned, long long unsigned > > > sameAddress_AtMultiClustersAndTime;
  std::unordered_map< long long unsigned, unordered_map < unsigned, long long unsigned > > sameAddress_AtMultiClusters_OverTime;
  
  std::unordered_map< long long unsigned, unsigned > sameAddress_Total; // simplest

  std::unordered_map < unsigned, unordered_map < std::string, set < long long unsigned > > > sameMC_SameContent_DifferentAddresses; //
  std::unordered_map < unsigned, unordered_map < std::string, long long unsigned > > sameMC_SameContent_LastAccessTime; //
  std::unordered_map < unsigned, unordered_map < std::string, set < long long unsigned > > > sameMC_SameContent_OverTime;
  std::unordered_map < unsigned, unordered_map < std::string, unordered_map < long long unsigned, set < long long unsigned > > > > sameMC_SameContent_DifferentAddresses_OverTime;
  std::unordered_map < unsigned, unordered_map < std::string, set < unsigned > > > sameMC_SameContent_AtMultiClusters;
  std::unordered_map < unsigned, unordered_map < std::string, unordered_map < long long unsigned, set < unsigned > > > > sameMC_SameContent_DifferentAddresses_AtMultiClusters;
  // std::unordered_map< unsigned, unordered_map < std::string, set < pair < unsigned, long long unsigned > > > > sameMC_SameContent_AtMultiClustersAndTime;
  std::unordered_map < unsigned, unordered_map < std::string, unordered_map < unsigned, long long unsigned > > > sameMC_SameContent_AtMultiClusters_OverTime;
  // std::unordered_map< unsigned, unordered_map < std::string, unordered_map < long long unsigned, set < pair < unsigned, long long unsigned > > > > > sameMC_SameContent_DifferentAddresses_AtMultiClustersAndTime;
  std::unordered_map < unsigned, unordered_map < std::string, unordered_map < long long unsigned, unordered_map < unsigned, long long unsigned > > > > sameMC_SameContent_DifferentAddresses_AtMultiClusters_OverTime;

  std::unordered_map < unsigned, unordered_map < std::string, unsigned > > sameMC_SameContent_Total; // simplest
  
  std::unordered_map < unsigned, unordered_map < std::string, unsigned > > sameMC_ApproxContent_Total; // simplest

  std::unordered_map < unsigned, unordered_map < std::string, set < long long unsigned > > > sameMC_ApproxContent_DifferentAddresses;
  std::unordered_map < unsigned, unordered_map < std::string, long long unsigned > > sameMC_ApproxContent_LastAccessTime; //
  std::unordered_map < unsigned, unordered_map < std::string, set < unsigned > > > sameMC_ApproxContent_AtMultiClusters; //


  //// tracing group, for write back case 
  std::unordered_map < long long unsigned, std::string > contentOfAddressInitial;
  std::unordered_map < long long unsigned, unordered_map <std::string, long long unsigned > > contentOfAddressTracing;
  std::unordered_map < long long unsigned, std::string > approxContentOfAddressInitial;
  std::unordered_map < long long unsigned, unordered_map <std::string, long long unsigned > > approxContentOfAddressTracing;
  std::unordered_map < long long unsigned, unsigned > mcOfAddress;

  std::set < long long unsigned > repeatedAccessAddresses;
  std::set < long long unsigned > repeatedAccessAddresses_100;
  std::set < long long unsigned > repeatedAccessAddresses_200;
  std::set < long long unsigned > repeatedAccessAddresses_ExactContent;
  std::set < long long unsigned > repeatedAccessAddresses_ExactContent_100;
  std::set < long long unsigned > repeatedAccessAddresses_ExactContent_200;
  std::set < long long unsigned > repeatedAccessAddresses_ApproxContent;
  std::set < long long unsigned > repeatedAccessAddresses_ApproxContent_100;
  std::set < long long unsigned > repeatedAccessAddresses_ApproxContent_200;
  ////




protected:
  
  class _BoundaryBufferItem {
  public:
    _BoundaryBufferItem():_packet_n(0) {}
    inline unsigned Size(void) const { return _buffer.size(); }
    inline bool HasPacket() const { return _packet_n; }
    void* PopPacket();
    void* TopPacket() const;
    // void PushFlitData(void* data, bool is_tail, unsigned source); // Khoa, 2022/07
    void PushFlitData(void* data, bool is_tail);
    
  private:
    queue<void *> _buffer;
    queue<bool> _tail_flag;
    int _packet_n;
  };
  typedef queue<Flit*> _EjectionBufferItem;
  

  void _CreateBuffer( );
  void _CreateNodeMap(unsigned n_shader, unsigned n_mem, unsigned n_node, int use_map);
  void _DisplayMap(int dim,int count);
  
  // size: [subnets][nodes][vcs]
  vector<vector<vector<_BoundaryBufferItem> > > _boundary_buffer;
  unsigned int _boundary_buffer_capacity;
  // size: [subnets][nodes][vcs]
  vector<vector<vector<_EjectionBufferItem> > > _ejection_buffer;
  // size:[subnets][nodes]
  vector<vector<queue<Flit* > > > _ejected_flit_queue;
  
  unsigned int _ejection_buffer_capacity;
  unsigned int _input_buffer_capacity;
  
  vector<vector<int> > _round_robin_turn; //keep track of _boundary_buffer last used in icnt_pop
  
  // GPUTrafficManager* _traffic_manager;//
  unsigned _flit_size;
  IntersimConfig* _icnt_config;
  unsigned _n_shader, _n_mem;
  vector<Network *> _net;
  int _vcs;
  int _subnets;
  
  //deviceID to icntID map
  //deviceID : Starts from 0 for shaders and then continues until mem nodes
  //which starts at location n_shader and then continues to n_shader+n_mem (last device)
  map<unsigned, unsigned> _node_map;
  
  //icntID to deviceID map
  map<unsigned, unsigned> _reverse_node_map;

};

#endif


