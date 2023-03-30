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

#ifndef _GPUTRAFFICMANAGER_HPP_
#define _GPUTRAFFICMANAGER_HPP_

#include <iostream>
#include <iomanip>

#include <sstream>
#include <fstream>
#include <limits> 
#include <cmath>
#include <string>
#include <list>
#include <unordered_map>
#include <vector>
#include <set>
#include <iterator>
#include <utility>
#include <deque>
#include <inttypes.h>

#include "config_utils.hpp"
#include "stats.hpp"
#include "trafficmanager.hpp"
#include "booksim.hpp"
#include "booksim_config.hpp"
#include "flit.hpp"

#include "../gpgpu-sim/mem_fetch.h"
#include "../cuda-sim/memory.h"
#include "interconnect_interface.hpp"




/// Khoa, 2022/07/
struct LUTRecord {
   
  bool _isRoot;
  // bool _isMe;
  long long unsigned _rootAddress; // single root/redirect address design
  int _sharerIcntID; // single sharer design
  // pair < long long unsigned, int > _sharingInfo;
  long long unsigned _entryTime;
  long long unsigned _lastChangeTime;
  int _lastChangerIcntID;
  // set<unsigned> _icntIDs; // holderIcntIDs
  // deque< pair< unsigned, long long unsigned > > _records; // holderIcntIDs, timeStamp
  // set< pair< unsigned, long long unsigned > > _records; // holderIcntIDs, timeStamp
  std::unordered_map < unsigned, long long unsigned > _sharers; // holderIcntIDs, timeStamp
  // std::unordered_map < long long unsigned, std::unordered_map < unsigned, long long unsigned > > _records; // approx address -> holderIcntIDs -> timeStamp
  std::unordered_map < long long unsigned, long long unsigned > _redirectAddresses; // approx address -> time stamp

  LUTRecord() {
    resetRecord();    
  }

  LUTRecord(int icntID, long long unsigned timeStamp = 0, bool isRoot = false, long long unsigned rootAddress = 0, bool isMe = false) {
    resetRecord();

    insertNode(icntID, timeStamp, -1);
    _isRoot = isRoot;
    // _isMe = isMe;
    _entryTime = timeStamp;
    _lastChangeTime = timeStamp;
    // if (!isRoot) {
      _rootAddress = rootAddress;
      if (_rootAddress != 0) {
        _redirectAddresses[_rootAddress] = timeStamp;
      }
    // }
  }

  ~LUTRecord() {

  }

  void resetRecord() {
    _isRoot = false;
    // _isMe = false;
    _rootAddress = 0;
    _sharerIcntID = -1;
    
    // _sharingInfo = make_pair(0, -1);
    // _icntIDs.clear();
    _sharers.clear();
    _redirectAddresses.clear();
    _entryTime = 0;
    _lastChangeTime = 0;
    _lastChangerIcntID = -1;
  }


  void insertSharingInfo(int holderIcntID, long long unsigned timeStamp, int changerIcntID) {
    changeNode(holderIcntID, timeStamp, changerIcntID);
    changeRedirectAddress(0, timeStamp, changerIcntID);
  }
  void changeSharingInfo(long long unsigned rootAddress, int holderIcntID, long long unsigned timeStamp, int changerIcntID) {
    changeRedirectAddress(rootAddress, timeStamp, changerIcntID);
    changeNode(holderIcntID, timeStamp, changerIcntID);
    // _sharingInfo = make_pair(rootAddress, holderIcntID);
    // _lastChangeTime = timeStamp;
  }
  // void clearSharingInfo(long long unsigned timeStamp) {
  //   _sharingInfo = make_pair(0, -1);
  //   _lastChangeTime = timeStamp;
  // }

  void insertRedirectAddress(long long unsigned rootAddress, long long unsigned timeStamp, int changerIcntID) {
    _redirectAddresses[rootAddress] = timeStamp;
    _lastChangeTime = timeStamp;
    _lastChangerIcntID = changerIcntID;
  }
  void changeRedirectAddress(long long unsigned rootAddress, long long unsigned timeStamp, int changerIcntID) {
    /// only for single redirect/root address design
    if (rootAddress == 0) {
      _isRoot = true;
    } else {
      _isRoot = false;
    }
    _rootAddress = rootAddress;
    _lastChangeTime = timeStamp;
    _lastChangerIcntID = changerIcntID;
  }
  void clearRedirectAddresses(long long unsigned timeStamp, int changerIcntID) {
    _redirectAddresses.clear();
    _lastChangeTime = timeStamp;
    _lastChangerIcntID = changerIcntID;
  }

  void insertNode(int holderIcntID, long long unsigned timeStamp, int changerIcntID) {
    // _icntIDs.insert( holderIcntID );
    // _records.push_back( make_pair(icntID, timeStamp) );
    // _records.insert( make_pair(icntID, timeStamp) );
    _sharers[holderIcntID] = timeStamp;
    _lastChangeTime = timeStamp;
    _lastChangerIcntID = changerIcntID;
    g_icnt_interface->totalLUTWrite_Homebase++;
  }
  void changeNode(int holderIcntID, long long unsigned timeStamp, int changerIcntID) {
    // only for single sharer design
    _sharerIcntID = holderIcntID;
    _lastChangeTime = timeStamp;
    _lastChangerIcntID = changerIcntID;
    g_icnt_interface->totalLUTWrite_Homebase++;
  }
  void clearNodes(long long unsigned timeStamp, int changerIcntID) {
    // _icntIDs.clear();
    _sharers.clear();
    _lastChangeTime = timeStamp;
    _lastChangerIcntID = changerIcntID;
  }

  unsigned size() {
    return _sharers.size();
    // return _icntIDs.size(); 
  }

}; 

struct LUTRecord_mP {

  LUTRecord_mP() {
    resetRecord();
  }

  ~LUTRecord_mP() {

  }

  void resetRecord() {
    _sharingAddresses.clear();
    _firstEntryTime = 0;
    _lastChangeTime = 0;
    // _sharingLimit = 1;
  }

  unordered_map < long long unsigned, long long unsigned > _sharingAddresses; // block address -> changed time
  long long unsigned _firstEntryTime; // when entry first added to look up table (LUT)
  long long unsigned _lastChangeTime; // also last access time
  unsigned _lastSharerIcntID; // maybe for later, currently unused
  long long unsigned _lastSharingAddress;
  // unsigned _sharingLimit;

  /*long long unsigned*/ void insertSharingAddress(long long unsigned blockAddress, long long unsigned curTime, unsigned holderIcntID) {
    if (_sharingAddresses.size() == 0) {
      _firstEntryTime = curTime;
    }
    _lastChangeTime = curTime;
    _sharingAddresses[blockAddress] = curTime;
    
    _lastSharerIcntID = holderIcntID;
    _lastSharingAddress = blockAddress;
    
    g_icnt_interface->totalLUTWrite_MC++;
    enforceSharingLimit(curTime); // return enforceSharingLimit(curTime);
  }


  long long unsigned enforceSharingLimit(long long unsigned curTime); 



};

////
// class LUTEntry {
//   public:
//     LUTEntry() {

//     }

//     LUTEntry(unsigned holderIcntID, long long unsigned entryTime, bool isMe, bool isRoot, long long unsigned rootAddress) {

//     }

//     ~LUTEntry() {

//     }

//     bool _isMe; // held by me, i.e., current node
//     bool _isRoot; // is root, i.e., 1st address of this content/approx content
//     long long unsigned _rootAddress; // if isMe => isRoot will have rootAddress == mf address, 
//     // and !isRoot will have rootAddress different from mf address
//     // if !isMe, then it does not know, but we can put it in just for the modifying data at mf address by the approx function

//     unsigned _holderNode; // current/last holder of content
//     long long unsigned _entryTime; // sim cycle that entry was added

// };
////



class GPUTrafficManager : public TrafficManager {
  
protected:
  virtual void _RetireFlit( Flit *f, int dest );
  virtual void _GeneratePacket(int source, int stype, int cl, int time, int subnet, int package_size, const Flit::FlitType& packet_type, void* const data, int dest);
  virtual int  _IssuePacket( int source, int cl );
  virtual void _Step(long long unsigned curCycle); // Khoa
  // virtual void _Step(); // Khoa
  
  // record size of _partial_packets for each subnet
  vector<vector<vector<list<Flit *> > > > _input_queue;
  
public:
  
  GPUTrafficManager( const Configuration &config, const vector<Network *> & net, unsigned n_shader );
  virtual ~GPUTrafficManager( );
  
  // correspond to TrafficManger::Run/SingleSim
  void Init();
  
  // TODO: if it is not good...
  friend class InterconnectInterface;
  
  
  
  //    virtual void WriteStats( ostream & os = cout ) const;
  //    virtual void DisplayStats( ostream & os = cout ) const;
  //    virtual void DisplayOverallStats( ostream & os = cout ) const;
  

  // Khoa, 2022/07

  unsigned printDeadlock;
  bool printFirst;
  unsigned debugCounter;
  unsigned printCounter;

  unsigned _n_shader;
  unsigned _entryLimit_cluster;
  unsigned _entryLimit_mP;
  unsigned _addressesPerMPEntry;
  bool _contentSharing_MC_serve_newAddress;
  bool _use_address_sharing_only;
  unsigned _use_approx_sharing;
  unsigned _use_approx_method;
  
  std::vector<unsigned> redirectedReadReq;
  unsigned _majorDim;
  InterconnectInterface* _m_icnt_i; // 
  // std::unordered_map<std::string, set<unsigned > > NoCLUT;
  // std::unordered_map<unsigned long long, set<unsigned > > NoCLUT_a;
  // std::vector< std::unordered_map<std::string, std::unordered_map < long long unsigned, std::unordered_map< unsigned, long long unsigned > > > > NoCLUT_memPart;
  // std::vector< std::unordered_map<long long unsigned, set< unsigned > > > NoCLUT_memPart;
  // std::vector< std::unordered_map<std::string, set< pair<unsigned, long long unsigned> > > > NoCLUT_memPart;
  // std::vector< std::unordered_map<std::string, set<unsigned > > > NoCLUT_memPart;
  
  
  // std::vector < 
  //   std::unordered_map < 
  //     std::string, 
  //     std::unordered_map < long long unsigned, 
  //       pair < 
  //         unsigned, 
  //         long long unsigned 
  //       > 
  //     > 
  //   > 
  // > NoCLUT_memPart; // NoCLUT_memPart: each node -> content tag -> address -> holderIcntID, time
  /// multiple root addresses design, each root address may be held by multiple sharers

  // std::vector < 
  //   std::unordered_map < 
  //     std::string, 
  //     pair < 
  //       long long unsigned, 
  //       long long unsigned 
  //     > 
  //   > 
  // > NoCLUT_memPart; // NoCLUT_memPart: each node -> content tag -> (address, time)
  // std::vector < 
  //   std::unordered_map < 
  //     std::string, 
  //     unordered_map < 
  //       long long unsigned, 
  //       long long unsigned 
  //     > 
  //   > 
  // > NoCLUT_memPart; // NoCLUT_memPart: each node -> content tag -> address -> time
  std::vector < 
    std::unordered_map < 
      std::string, 
      LUTRecord_mP
    > 
  > NoCLUT_memPart; // NoCLUT_memPart: each node -> content tag -> LUTRecord_mP (address, time, holder,...)
  /// single root address design, single address (the first one accessed) for each content set, 
  /// at each later access, update previous address's homebase, turning it into non-root, 
  /// and wait for receiver to update the new address's homebase   

  std::vector < 
    std::unordered_map < 
      long long unsigned, 
      LUTRecord 
    > 
  > NoCLUT_cluster; // NoCLUT_cluster: each node -> address -> record (holderIcntID, time)

  // std::vector< std::unordered_map<std::string, unsigned > > NoCLUT_memPart_s;
  // std::vector< std::unordered_map<std::string, pair< unsigned, long long unsigned> > > NoCLUT_memPart_s;  
  // std::vector< std::unordered_map< long long unsigned, unsigned > > NoCLUT_cluster_s;
  

  void printLUT_test(unsigned currentNode, Flit* f, long long unsigned curTime, std::string extraText);


  // void add_LUT_entry(std::string dataKey, unsigned holderNode);
  // void add_LUT_entry(long long unsigned blockAddress, unsigned holderNode);
  void add_LUT_entry_memPart(unsigned currentNode, std::string dataKey, unsigned holderNode, long long unsigned address);
  // void add_LUT_entry_memPart(unsigned currentNode, std::string dataKey, unsigned holderNode);
  // void add_LUT_entry_cluster(unsigned currentNode, bool isMe, bool isRoot, unsigned long long blockAddress, unsigned holderNode);
  void add_LUT_entry_cluster(unsigned currentNode, long long unsigned blockAddress, unsigned holderNode, bool isRoot, long long unsigned rootAddress);

  long long unsigned enforceLUTEntryLimit_cluster(unsigned nodeIcntID);
  std::string enforceLUTEntryLimit_memPart(unsigned nodeIcntID);
  void removeLUTEntry_cluster(unsigned nodeIcntID, long long unsigned address);
  void removeLUTEntry_memPart(unsigned nodeIcntID, std::string dataKey);

  bool checkLUTEntry_cluster(unsigned nodeIcntID, long long unsigned blockAddress);
  bool checkLUTEntry_memPart(unsigned nodeIcntID, std::string dataKey);
  bool reroutePacket(unsigned nodeIcntID, Flit* f);


  std::string makeDataKey(mem_fetch* mf, std::string & exactDataKey);
  std::string makeDataKey_Approx(mem_fetch* mf, unsigned wordByteSize, unsigned bitMask);
  std::string makeDataKey_Approx_Average(mem_fetch* mf, unsigned wordByteSize, unsigned binaryMode);

  bool compareApprox(mem_fetch* mf, long long unsigned targetAddress);

  unsigned computeManhDistance(unsigned majorDimension, unsigned node1, unsigned node2);

  unsigned estimateFullTripCost(unsigned majorDimension, unsigned source_node, unsigned destination_node, unsigned current_node);

};



#endif
