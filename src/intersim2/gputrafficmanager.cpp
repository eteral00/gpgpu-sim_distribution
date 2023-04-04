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

#include <sstream>
#include <fstream>
#include <limits> 
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <set>
#include <cmath>
#include <iterator>
// #include <inttypes.h>

#include "gputrafficmanager.hpp"
#include "interconnect_interface.hpp"
#include "globals.hpp"

#include "../gpgpu-sim/mem_fetch.h"
#include "../cuda-sim/memory.h"
#include "../cuda-sim/half.h"


// void printBits(std::string filePath, size_t const size, void const * const ptr)
// {
//     FILE *resOutFile_ = fopen(filePath.c_str(), "a");  
//     unsigned char *b = (unsigned char*) ptr;
//     unsigned char byte;
//     int i, j;
    
//     for (i = size-1; i >= 0; i--) {
//         fprintf(resOutFile_, " ");
//         for (j = 7; j >= 0; j--) {
//             byte = (b[i] >> j) & 1;
//             fprintf(resOutFile_, "%u", byte);
//         }
        
//         if ( (i % 4) == 0) {
//           fprintf(resOutFile_, ", ");
//         }
//     }
//     fprintf(resOutFile_, ",");
//     // fprintf(resOutFile_, "\n");
//     // puts("");
//     fclose(resOutFile_);
// }


// Khoa, 2022/07/

void GPUTrafficManager::printLUT_test(unsigned currentNode, Flit* f, long long unsigned curTime, std::string extraText ) {
  
  std::string filePath = "testNoCLUT_cluster_addEntry_" + to_string(currentNode) + "_.csv";
  
  mem_fetch* mf = static_cast<mem_fetch* >(f->data);
  long long unsigned mfBlAddress = mf->get_addr();
  std::string exactDataKey;
  std::string dataKey = makeDataKey(mf, exactDataKey);
  
  unsigned homebaseIcntID = g_icnt_interface->getHomebase(mfBlAddress);
  unsigned redirectAddressHomebaseIcntID = g_icnt_interface->getHomebase(mf->redirectedAddress);
  int sharerIcntID = -1;
  long long unsigned lastChangedTime = 0;
  int lastChangerIcntID = -1;

  if (NoCLUT_cluster[homebaseIcntID].count(mfBlAddress)) {
    sharerIcntID = NoCLUT_cluster[homebaseIcntID][mfBlAddress]._sharerIcntID;
    lastChangedTime = NoCLUT_cluster[homebaseIcntID][mfBlAddress]._lastChangeTime;
    lastChangerIcntID = NoCLUT_cluster[homebaseIcntID][mfBlAddress]._lastChangerIcntID;
  }
  FILE *resOutFile_cm = fopen(filePath.c_str(), "a");
  
  fprintf(resOutFile_cm,
    "0x%s,0x%08llx__%u,0x%08llx__%u,%u__%u__%u,%u|%d,%llu,%s,%llu,%d\n",
      dataKey.c_str(),
      mfBlAddress,
      homebaseIcntID,
      mf->redirectedAddress,
      redirectAddressHomebaseIcntID,
      f->dest,
      f->destMC,
      f->src,
      f->isProbeFlag,
      sharerIcntID,
      curTime,
      extraText.c_str(),
      lastChangedTime,
      lastChangerIcntID
  );

  fclose(resOutFile_cm);

}


void GPUTrafficManager::add_LUT_entry_memPart(unsigned currentNode, std::string dataKey, unsigned holderNode, long long unsigned address) {

  //// add entry
  // NoCLUT_memPart[currentNode][dataKey].insert(holderNode);
  // NoCLUT_memPart[currentNode][dataKey].insert( make_pair(holderNode, address) );
  // NoCLUT_memPart[currentNode][address].insert( holderNode );

  NoCLUT_memPart[currentNode][dataKey].insertSharingAddress(address, getTime(), holderNode);
  enforceLUTEntryLimit_memPart(currentNode);

  // NoCLUT_memPart[currentNode][dataKey][address] = getTime(); // add new sharer
  // if (NoCLUT_memPart[currentNode][dataKey].size() > 4) { /// current limit sharers = 4
  //   unordered_map < long long unsigned, long long unsigned >::iterator itrt_memPart;
  //   long long unsigned oldestTime = getTime();
  //   long long unsigned oldestAddress = 0;
  //   for (itrt_memPart = NoCLUT_memPart[currentNode][dataKey].begin();
  //         itrt_memPart != NoCLUT_memPart[currentNode][dataKey].end();
  //         itrt_memPart++) {
  //     /// search for oldest sharer
  //     if (itrt_memPart->second < oldestTime) {
  //       oldestTime = itrt_memPart->second;
  //       oldestAddress = itrt_memPart->first;
  //     }
  //   }
  //   NoCLUT_memPart[currentNode][dataKey].erase(oldestAddress); // remove oldest sharer
  // }
  
  //// tracking test
  // if (currentNode == 50) {
  //   std::string filePath = "testNoCLUT_memPart_addEntry_" + to_string(currentNode) + "_.csv";
  //   FILE *resOutFile_cm = fopen(filePath.c_str(), "a");
  //   fprintf(resOutFile_cm,
  //     "0x%s,0x%08llx__%u,%llu\n",
  //       dataKey.c_str(),
  //       address,
  //       g_icnt_interface->getHomebase(address),
  //       getTime()
  //   );
  //   fclose(resOutFile_cm);
  // }
  ////

  // FILE *resOutFile_ma = fopen("testNoCLUT_mem_add_.csv", "a");
  // fprintf(resOutFile_ma, "cluster node# %u|%u,%u,", 
  //   currentNode,
  //   NoCLUT_memPart_s.size(), 
  //   NoCLUT_memPart_s[currentNode].size()
  // );

  // NoCLUT_memPart[currentNode][dataKey].insert( holderNode );
  
  // fprintf(resOutFile_ma, "%u,%s,%u\n",  
  //     NoCLUT_memPart_s[currentNode].size(),
  //     dataKey.c_str(),
  //     holderNode
  //     );

  // fclose(resOutFile_ma);


  // NoCLUT_memPart_s[currentNode][dataKey] = holderNode;
  // NoCLUT_memPart_s[currentNode][dataKey] = make_pair(holderNode, address);
  // if (NoCLUT_memPart_s[currentNode].count(dataKey)) {
  //   NoCLUT_memPart_s[currentNode][dataKey] = holderNode;
    
  //   if (currentNode == 60) {
  //     FILE *resOutFile_m60 = fopen("testNoCLUT_mem_60_e_.csv", "a");
  //     fprintf(resOutFile_m60, "cluster node# %u|%u,%u,,%s,%u\n", 
  //       currentNode,
  //       NoCLUT_memPart_s.size(), 
  //       NoCLUT_memPart_s[currentNode].size(),
  //       dataKey.c_str(),
  //       holderNode
  //     );

  //     fclose(resOutFile_m60);
  //   }

  // } else {
    
  //   if (g_icnt_interface->getDeviceID(currentNode) >= _n_shader) {
  //     FILE *resOutFile_ma = fopen("testNoCLUT_mem_add_.csv", "a");
  //     fprintf(resOutFile_ma, "cluster node# %u|%u,%u,", 
  //       currentNode,
  //       NoCLUT_memPart_s.size(), 
  //       NoCLUT_memPart_s[currentNode].size()
  //     );

  //     NoCLUT_memPart_s[currentNode][dataKey] = holderNode;
      
  //     fprintf(resOutFile_ma, "%u,%s,%u\n",  
  //         NoCLUT_memPart_s[currentNode].size(),
  //         dataKey.c_str(),
  //         holderNode
  //         );

  //     fclose(resOutFile_ma);
  //   } else {
  //     NoCLUT_memPart_s[currentNode][dataKey] = holderNode;
  //   }
  // }
}


//// not used in current version
void GPUTrafficManager::add_LUT_entry_cluster(unsigned currentNode, unsigned long long blockAddress, unsigned holderNode, bool isRoot, long long unsigned rootAddress = 0) {
  long long unsigned curTime = getTime();
  
  //// always update sharer ID
  NoCLUT_cluster[currentNode][blockAddress].changeNode((int)holderNode, curTime, -1); //// single-sharer design
  
  if (isRoot) {
    //// is root entry, update root address to '0'
    NoCLUT_cluster[currentNode][blockAddress].changeRedirectAddress(0, curTime, -1);

    // NoCLUT_cluster[currentNode][blockAddress].insertNode(holderNode, curTime); //// multi-sharer design
    
  } else {
    //// is non-root entry, update redirect addresses
    // assert(rootAddress != 0);
    // NoCLUT_cluster[currentNode][blockAddress].changeRedirectAddress(rootAddress, curTime, -1);
    // NoCLUT_cluster[currentNode].erase(blockAddress); // keep only last info
    // g_icnt_interface->totalLUTWrite_Homebase++;
    ////
    
    // if (g_icnt_interface->getHomebase(rootAddress) == currentNode) {
    //   /// check isMe, only if sharer is not included with non-root address 
    //   NoCLUT_cluster[currentNode][blockAddress]._isMe = true; // isMe means the root address's homebase is also at current node
    // } else {
    //   NoCLUT_cluster[currentNode][blockAddress]._isMe = false;
    // }
  }
  //// enforce entry limit
  enforceLUTEntryLimit_cluster(currentNode);



  // // NoCLUT_cluster_s[currentNode][blockAddress] = holderNode;
  // // if (NoCLUT_cluster_s[currentNode].count(blockAddress)) {
  // //   NoCLUT_cluster_s[currentNode][blockAddress] = holderNode;
    
  // //   if (currentNode == 60) {
  // //     FILE *resOutFile_c60 = fopen("testNoCLUT_cluster_60_e_.csv", "a");
  // //     fprintf(resOutFile_c60, "cluster node# %u|%u,%u,,%llx,%u\n", 
  // //       currentNode,
  // //       NoCLUT_cluster_s.size(), 
  // //       NoCLUT_cluster_s[currentNode].size(),
  // //       blockAddress,
  // //       holderNode
  // //     );

  // //     fclose(resOutFile_c60);
  // //   }

  // // } else {
    
  // //   if (g_icnt_interface->getDeviceID(currentNode) >= _n_shader) {
  // //     FILE *resOutFile_ca = fopen("testNoCLUT_cluster_add_.csv", "a");
  // //     fprintf(resOutFile_ca, "cluster node# %u|%u,%u,", 
  // //       currentNode,
  // //       NoCLUT_cluster_s.size(), 
  // //       NoCLUT_cluster_s[currentNode].size()
  // //     );

  // //     NoCLUT_cluster_s[currentNode][blockAddress] = holderNode;
      
  // //     fprintf(resOutFile_ca, "%u,%llx,%u\n",  
  // //         NoCLUT_cluster_s[currentNode].size(),
  // //         blockAddress,
  // //         holderNode
  // //         );

  // //     fclose(resOutFile_ca);
  // //   } else {
  // //     NoCLUT_cluster_s[currentNode][blockAddress] = holderNode;
  // //   }

  // //   // if (NoCLUT_cluster_s[currentNode].size() < 100) {
  // //   //   NoCLUT_cluster_s[currentNode][blockAddress] = holderNode;
  // //   // }
  // // }
  
  
  // // if (printCounter < 64) {  
  // //   cout << "TEST::NODE# " << currentNode << ":" 
  // //       << "size: " << NoCLUT_cluster_s[currentNode].size() << ", "
  // //       << "holder: " << NoCLUT_cluster_s[currentNode][blockAddress] << ", " 
  // //       << "iter: " << NoCLUT_cluster_s[currentNode].begin()->second << "\n" ;
  // //   printCounter++;
  // // }


  // long long unsigned curSimTime = getTime();
  // FILE *resOutFile_ca = fopen("testNoCLUT_cluster_add_.csv", "a");
  // fprintf(resOutFile_ca, "cluster node# %u|%u,%u,", 
  //   currentNode,
  //   NoCLUT_cluster.size(), 
  //   NoCLUT_cluster[currentNode].size()
  // );

  // NoCLUT_cluster[currentNode][blockAddress].insertNode(holderNode, curSimTime);
  
  // fprintf(resOutFile_ca, "%u,0x%08llx,%u,%llu\n",  
  //     NoCLUT_cluster[currentNode].size(),
  //     blockAddress,
  //     holderNode,
  //     curSimTime
  //     );

  // fclose(resOutFile_ca);
  
  // // if (NoCLUT_cluster[currentNode].count(blockAddress)) {
  // //   // record already exists for this block
  // //   // NoCLUT_cluster[currentNode][blockAddress]._icntIDs.clear(); // clear record
  // //   if (!NoCLUT_cluster[currentNode][blockAddress]._isMe) {
  // //     // block holder list does not include self
  // //     if (isMe) {
  // //       // adding self
  // //       NoCLUT_cluster[currentNode][blockAddress]._isMe = isMe;
  // //       NoCLUT_cluster[currentNode][blockAddress]._isRoot = isRoot;

  // //       if (!isRoot) {
  // //         NoCLUT_cluster[currentNode][blockAddress]._rootAddress = blockAddress;
  // //       }
      
  // //     } //else {
  // //       // adding another peer
  // //       NoCLUT_cluster[currentNode][blockAddress].insertNode(holderNode);

  // //     //}

  // //   } // else, self, i.e., current node has this block, ignore
  
  // // } else {
  // //   // record does not exist for this block address

  // //   NoCLUT_cluster[currentNode][blockAddress]._isMe = isMe;
  // //   NoCLUT_cluster[currentNode][blockAddress]._isRoot = isRoot;

  // //   if (isMe && !isRoot) {
  // //     NoCLUT_cluster[currentNode][blockAddress]._rootAddress = blockAddress;
  // //   }

  // //   // if (!isMe) {
  // //     NoCLUT_cluster[currentNode][blockAddress].insertNode(holderNode);
  // //   // }

  // // }
}

long long unsigned GPUTrafficManager::enforceLUTEntryLimit_cluster(unsigned nodeIcntID) {
  long long unsigned oldestAddress = 0;

  if (_entryLimit_cluster && NoCLUT_cluster[nodeIcntID].size() > _entryLimit_cluster ) {
    long long unsigned oldestTime = getTime(); // current time
    unordered_map < long long unsigned, LUTRecord >::iterator itrt;
    
    for ( itrt = NoCLUT_cluster[nodeIcntID].begin();
          itrt != NoCLUT_cluster[nodeIcntID].end();
          itrt++ ) {
      if (itrt->second._lastChangeTime < oldestTime) {
        oldestTime = itrt->second._lastChangeTime;
        oldestAddress = itrt->first;
      }
    }

    if (oldestAddress) {
      removeLUTEntry_cluster(nodeIcntID, oldestAddress);
    }
  }

  return oldestAddress;
}

std::string GPUTrafficManager::enforceLUTEntryLimit_memPart(unsigned nodeIcntID) {
  std::string oldestDataKey = "";

  if (_entryLimit_mP && NoCLUT_memPart[nodeIcntID].size() > _entryLimit_mP) {
    long long unsigned oldestTime = getTime(); // current
    unordered_map < std::string, LUTRecord_mP>::iterator itrt;

    for ( itrt = NoCLUT_memPart[nodeIcntID].begin();
          itrt != NoCLUT_memPart[nodeIcntID].end();
          itrt++ ) {
      if (itrt->second._lastChangeTime < oldestTime) {
        oldestTime = itrt->second._lastChangeTime;
        oldestDataKey = itrt->first;
      }
    }
    
  }

  if (oldestDataKey != "") {
    unordered_map < long long unsigned, long long unsigned >::iterator itrt_sharingAddresses;
    // for ( itrt_sharingAddresses = NoCLUT_memPart[nodeIcntID][oldestDataKey]._sharingAddresses.begin();
    //       itrt_sharingAddresses != NoCLUT_memPart[nodeIcntID][oldestDataKey]._sharingAddresses.end();
    //       itrt_sharingAddresses++ ) {
    //   unsigned homebaseIcntID = g_icnt_interface->getHomebase(itrt_sharingAddresses->first);
    //   removeLUTEntry_cluster(homebaseIcntID, itrt_sharingAddresses->first);
    // } // Khoa, 2023/02/23, leave old/un-updated info at clusters' LuTs and let each cluster remove them when full 
    removeLUTEntry_memPart(nodeIcntID, oldestDataKey);
  }

  return oldestDataKey;
}

void GPUTrafficManager::removeLUTEntry_cluster(unsigned nodeIcntID, long long unsigned address) {
  g_icnt_interface->totalLUTWrite_Homebase++;
  NoCLUT_cluster[nodeIcntID].erase(address);
}

void GPUTrafficManager::removeLUTEntry_memPart(unsigned nodeIcntID, std::string dataKey) {
  g_icnt_interface->totalLUTWrite_MC++;
  NoCLUT_memPart[nodeIcntID].erase(dataKey);
}


bool GPUTrafficManager::checkLUTEntry_cluster(unsigned curIcntID, long long unsigned blockAddress) {
  g_icnt_interface->totalLUTRead_Homebase++;
  return NoCLUT_cluster[curIcntID].count(blockAddress);
}
bool GPUTrafficManager::checkLUTEntry_memPart(unsigned curIcntID, std::string dataKey) {
  g_icnt_interface->totalLUTRead_MC++;
  return NoCLUT_memPart[curIcntID].count(dataKey);
}

bool GPUTrafficManager::reroutePacket(unsigned curIcntID, Flit* f) {
  bool isRedirected = false;
  long long unsigned curTime = getTime();
  mem_fetch* mft = static_cast<mem_fetch* >(f->data);
  long long unsigned blockAddress = mft->get_addr();
  std::string exactDataKey;
  std::string dataKey = makeDataKey(mft, exactDataKey);
  // unsigned estimatedCost = estimateFullTripCost(8, f->src, f->dest, curIcntID); // hard-coded major dimension = 8
  // unsigned tCost;
  // unsigned bestDest = f->dest;
  // long long unsigned redirectedAddress;
  // bool approxMatch = false;
  long long unsigned sharingAddress;
  unsigned homebaseIcntID;

  if (dataKey != "") {
    if (f->head && f->type == Flit::READ_REQUEST) {
      //// only reroute read request
      if (f->dest == curIcntID) {
        //// checking f's target homebase == current node
        if (f->isProbeFlag) {
          //// replace f's redirectedFlag with isProbeFlag when change from MC/centralized to cluster/distributed
          //// f is probe, checking look up table (LUT)
          //// asserting current node is correct homebase for said address
          assert(curIcntID == g_icnt_interface->getHomebase(blockAddress));

          if ( checkLUTEntry_cluster(curIcntID, blockAddress) ) {
            //// found a sharer at homebase directory

            //// change current target dest to better dest
            // bestDest = NoCLUT_cluster[curIcntID][blockAddress]._sharerIcntID;
            f->dest = NoCLUT_cluster[curIcntID][blockAddress]._sharerIcntID; 
            
            //// tracking test
            // printLUT_test(curIcntID, f, curTime, "rf_cluster_probe_found_before_change");
            mft->sharerIcntID = f->dest; // for tracking
              
            if ( !(NoCLUT_cluster[curIcntID][blockAddress]._isRoot) ) {
              //// entry address is not root, redirect to sharer holding root address, 
              //// i.e., a different address with similar/approx content

              // redirectedAddress = NoCLUT_cluster[curIcntID][blockAddress]._rootAddress;
              sharingAddress = NoCLUT_cluster[curIcntID][blockAddress]._rootAddress;
              mft->redirectedAddress = sharingAddress; // for tracking
              
              if (_use_approx_sharing) {
                //// modifying actual data in memory and re-execute LOAD instruction
                // FILE *resOutFile_tapp = fopen("testApproxSharing_.csv", "a");
                // fprintf( resOutFile_tapp, 
                //   "\n USE APPROX SHARING, CHANGE_MF_DATA: %u \n", 
                //   _use_approx_method
                //   );
                // fclose(resOutFile_tapp);

                g_icnt_interface->changeMfData(mft, f->src); // for approx content only
              } // APPROX CRITICAL
              

              //// update LUT (opportunistic way):  for homebase of root address
              //// need moving ? Khoa, 2023/02/09
              // homebaseIcntID = g_icnt_interface->getHomebase(sharingAddress);
              // g_icnt_interface->totalHops_Update += computeManhDistance(8, curIcntID, homebaseIcntID);
              // NoCLUT_cluster[homebaseIcntID][sharingAddress].changeNode(f->src, curTime, curIcntID);
              // enforceLUTEntryLimit_cluster(homebaseIcntID);
              /// moved, 2023/02/26
            }
              //// update LUT (opportunistic way):  for self/ current node (always, whether entry address was root or not)
              // NoCLUT_cluster[curIcntID][blockAddress].changeNode(f->src, curTime, curIcntID); // moved, 2023/02/26
              // NoCLUT_cluster[curIcntID].erase(blockAddress); // remove old entry, for only keep last info entry design
              ////

              isRedirected = true; // for statistics print out
              f->redirectedFlag = true;

              //// tracking test
              // if (curIcntID == 50) {
              //   printLUT_test(curIcntID, f, curTime, "rf_cluster_probe_found");
              // }
              ////

          } else {
            //// no entry found at homebase
            f->dest = f->destMC; // redirect to block address's MC
            // f->redirectedFlag = false;
          }

          f->isProbeFlag = false; // change to normal read request packet
        }
        //// if not probe, and use only address sharing, no redirect, i.e., MCs serve data
        //// here, not using address_sharing => mc redirects requests
        else if (_use_address_sharing_only == 0) {
          if (g_icnt_interface->getDeviceID(curIcntID) >= _n_shader) {
            //// not probe, i.e., normal read request arriving at destination
            //// and dest is a Memory Controller
            //// asserting packet arrive at correct MC, 
            //// for there is a (bug-)case that an address's homebase is a MC node but it's not the address's data-holding MC
            //// logically, if all parts are done correctly, that case will not happen, so when it does, something's wrong
            assert (f->destMC == curIcntID);

            if ( checkLUTEntry_memPart(curIcntID, dataKey) ) {
              /// found a previously accessed addresses with approx content

              //// modifying actual data in memory and reload
              //// only used if first access of an address is also redirected by MC, or replied with approx data
              //// not used when first access of an address is replied with exact data
              // mft->redirectedAddress = NoCLUT_memPart[curIcntID][dataKey].first; // set redirect address // old design

              mft->redirectedAddress = NoCLUT_memPart[curIcntID][dataKey]._lastSharingAddress; // set redirect address
              if (_use_approx_sharing) {
                g_icnt_interface->changeMfData(mft, f->src);
                
              } 
              //// 2023/03/ , update LuT_mem
              // NoCLUT_memPart[curIcntID][dataKey].insertSharingAddress(blockAddress, curTime, f->src); // moved
              ////
              ////

              // if (_contentSharing_MC_serve_newAddress) {
              //   //// 1st design: memory controller will handle and reply (with exact or approx data)
              //   //// we only need to handle the LUT update here
              // } else {
              //   //// 2nd design: redirect request to last sharer (likely, data will be approx)
              //   f->dest = NoCLUT_memPart[curIcntID][dataKey]._lastSharerIcntID;
              //   isRedirected = true; // for statistics print out
              //   f->redirectedFlag = true;
              //   ////
              // }
              

              // //// may move to when MC or cluster send reply
              // //// update multi roots
              // //// update homebases of previous sharing addresses
              // unordered_map< long long unsigned, long long unsigned >::iterator itrt_memPart;
              // for ( itrt_memPart = NoCLUT_memPart[curIcntID][dataKey]._sharingAddresses.begin();
              //       itrt_memPart != NoCLUT_memPart[curIcntID][dataKey]._sharingAddresses.end();
              //       itrt_memPart++ ) {
              //   sharingAddress = itrt_memPart->first;
              //   homebaseIcntID = g_icnt_interface->getHomebase(sharingAddress);

              //   //// current access's  block address will be new sharing address
              //   //// f's source, i.e., the requester, will be the new sharer
              //   g_icnt_interface->totalHops_Update += computeManhDistance(8, curIcntID, homebaseIcntID);
              //   NoCLUT_cluster[homebaseIcntID][sharingAddress].changeSharingInfo(blockAddress, f->src, curTime, curIcntID);
              //   enforceLUTEntryLimit_cluster(homebaseIcntID);
              //   // NoCLUT_cluster[homebaseIcntID].erase(sharingAddress); // remove old entry, for keep-only-last-info-entry design
              // }

            } else {
              //// no matching content from previous accesses
              //// reply will be exact data
            } //// Khoa, 2023/03/31, moved check LuT_mem to when "replies" are sent

            ////// update homebase of new address (always) (also opportunistic way)
            // //// moving this to when MC sending reply
            // //// insert new entry at new address's homebase
            // homebaseIcntID = g_icnt_interface->getHomebase(blockAddress);
            // g_icnt_interface->totalHops_Update += computeManhDistance(8, curIcntID, homebaseIcntID);
            // NoCLUT_cluster[homebaseIcntID][blockAddress].insertSharingInfo(f->src, curTime, curIcntID);
            // enforceLUTEntryLimit_cluster(homebaseIcntID);

            // //// update LUT of memPart 
            // //// also moving this to when MC sending reply
            // //// for update if same content set accessed again later (same procedure, but different purpose from MC/centralized design)
            // add_LUT_entry_memPart(curIcntID, dataKey, f->src, blockAddress);
            //////

            //// tracking test
            // if (curIcntID == 50) {
            //   g_icnt_interface->_traffic_manager->printLUT_test(curIcntID, f, curTime, "rf_MC_req");
            // }
            ////

          } // end if (getDeviceID(curIcntID) >= _n_shader), i.e., isMC
        } // end if (probe) else if ()
      } // end if (dest == curIcntID)
    } // end if (head && read_request)
  } // end if (dataKey)
  return isRedirected;
}


std::string GPUTrafficManager::makeDataKey(mem_fetch* mf, std::string &exactDataKey) {
  // debugCounter++;
  // unsigned subCounter = 0;

  // FILE * resOutFile_makeDataKey = fopen("testMakeDataKey_n_.csv", "a");
  // fprintf(resOutFile_makeDataKey,
  //   "%llu,%u,%s",
  //     getTime(),
  //     debugCounter,
  //     noteMsg.c_str()
  // );
  // fclose(resOutFile_makeDataKey);

  long long unsigned my_addr;
  my_addr = mf->get_addr();
  memory_space *mem = NULL;
  memory_space_t m_m_space = mf->get_inst().space;

  // memory_space_t m_m_space = mf->get_inst().whichspace(my_addr);
  unsigned my_access_size = mf->get_access_size();
  char * tBuffer = new char[my_access_size+1];

  if(mf->get_w_inst()->decode_space(m_m_space, mem, my_addr)) {
    if (m_m_space == global_space) {
      mem->read(my_addr, my_access_size, tBuffer);
    } else {
      memset(tBuffer, 0xff, my_access_size);
    }
  } else {
    memset(tBuffer, 0xff, my_access_size);
  }

  tBuffer[my_access_size] = '\0';

  unsigned long long * packet_data_hex = new unsigned long long[my_access_size/8];
  
  memcpy( (void*)packet_data_hex, (void*)tBuffer, my_access_size );

  std::ostringstream strStream;  

  for (int idx = 0; idx < (my_access_size/8); idx++) {
    strStream << std::setfill('0') <<  std::hex << std::setw(8) << packet_data_hex[idx];
  }

  exactDataKey = strStream.str(); // return extra val through a reference, for comparision
  // std::string dataKey = strStream.str();

  delete []packet_data_hex;
  delete []tBuffer;

  // exactDataKey = dataKey; // return extra val through a reference, for comparison
  if (_use_approx_sharing == 0) {
    return strStream.str();
  } else {
    switch (_use_approx_method) {
      case 0:
        return makeDataKey_Approx(mf, 2, 0xFC00);
        break;
      case 1:
        return makeDataKey_Approx_Average(mf, 2, 0);
        break;
      case 2:
        return makeDataKey_Approx_Average(mf, 2, 1);
        break;
      default:
        return strStream.str();
        break;
    }
    
  }
  
}


std::string GPUTrafficManager::makeDataKey_Approx(mem_fetch* mf, unsigned wordByteSize = 2, unsigned bitMask = 0xFFFF) {
  std::string dataKey;
  unsigned long long int my_addr;
  my_addr = mf->get_addr();
  memory_space *mem = NULL;
  memory_space_t m_m_space = mf->get_inst().space;
  // memory_space_t m_m_space = mf->get_inst().whichspace(my_addr);
  
  unsigned my_access_size = mf->get_access_size();
  char * tBuffer = new char[my_access_size+1];

  if (mf->get_w_inst()->decode_space(m_m_space, mem, my_addr)) {
    if (m_m_space == global_space) {
      mem->read(my_addr, my_access_size, tBuffer);
    } else {
      memset(tBuffer, 0xf0, my_access_size);
    }

    tBuffer[my_access_size] = '\0';

    // if (wordByteSize == 2) {
    //   uint16_t * packet_data_hex = new uint16_t[my_access_size/wordByteSize];
    // } else if (wordByteSize == 4) {
    //   uint32_t * packet_data_hex = new uint32_t[my_access_size/wordByteSize];
    // } else {
    //   unsigned char * packet_data_hex = new unsigned char[my_access_size];
    // }
    
    uint16_t * packet_data_hex = new uint16_t[my_access_size/wordByteSize];
    vector< half_float::half > packet_data_half;
    packet_data_half.resize(my_access_size/wordByteSize);
    memcpy( (void*)packet_data_hex, (void*)tBuffer, my_access_size );

    //// approx by bit
    std::ostringstream strStream;  
    for (unsigned idx = 0; idx < (my_access_size/wordByteSize); idx++) {
      // approximating
      // packet_data_hex[idx] = g_icnt_interface->approx_Bit_16(packet_data_hex[idx], (uint16_t)bitMask);
      packet_data_hex[idx] = g_icnt_interface->approx_Bit_16_2Steps(packet_data_hex[idx], 15, 2);
      
      // packet_data_half[idx].data_ = packet_data_hex[idx];
      // packet_data_half[idx] = ceil(packet_data_half[idx]);
      
      // packet_data_hex[idx] = packet_data_half[idx].data_;
      // end approximating 
      strStream << std::setfill('0') <<  std::hex << std::setw(wordByteSize*2) << packet_data_hex[idx];
    }
    dataKey = strStream.str();
    ////

    //// approx average + range
    // dataKey = g_icnt_interface->approx_Average(packet_data_hex, (my_access_size/wordByteSize));
    ////

    delete []packet_data_hex;
  
  } else {
    dataKey = "";
  }

  delete []tBuffer;

  return dataKey;
}


std::string GPUTrafficManager::makeDataKey_Approx_Average(mem_fetch* mf, unsigned wordByteSize = 2, unsigned binaryMode = 0) {
  std::string dataKey;
  unsigned long long int my_addr;
  my_addr = mf->get_addr();
  memory_space *mem = NULL;
  memory_space_t m_m_space = mf->get_inst().space;
  // memory_space_t m_m_space = mf->get_inst().whichspace(my_addr);
  
  unsigned my_access_size = mf->get_access_size();
  char * tBuffer = new char[my_access_size+1];

  if (mf->get_w_inst()->decode_space(m_m_space, mem, my_addr)) {
    if (m_m_space == global_space) {
      mem->read(my_addr, my_access_size, tBuffer);
    } else {
      memset(tBuffer, 0xf0, my_access_size);
    }

    tBuffer[my_access_size] = '\0';

    // if (wordByteSize == 2) {
    //   uint16_t * packet_data_hex = new uint16_t[my_access_size/wordByteSize];
    // } else if (wordByteSize == 4) {
    //   uint32_t * packet_data_hex = new uint32_t[my_access_size/wordByteSize];
    // } else {
    //   unsigned char * packet_data_hex = new unsigned char[my_access_size];
    // }
    
    uint16_t * packet_data_hex = new uint16_t[my_access_size/wordByteSize];
    vector< half_float::half > packet_data_half;
    packet_data_half.resize(my_access_size/wordByteSize);
    memcpy( (void*)packet_data_hex, (void*)tBuffer, my_access_size );

    //// approx average + range
    if (binaryMode == 0) {
      dataKey = g_icnt_interface->approx_Average(packet_data_hex, (my_access_size/wordByteSize));
    } else if (binaryMode == 1) {
      dataKey = g_icnt_interface->approx_AverageBinary(packet_data_hex, (my_access_size/wordByteSize));
    }
    
    ////

    delete []packet_data_hex;
  
  } else {
    dataKey = "";
  }

  delete []tBuffer;

  return dataKey;
}


bool GPUTrafficManager::compareApprox(mem_fetch* mf, long long unsigned targetAddress) {
  bool result = false;
  AverageRangePair curPair;
  AverageRangePair targetPair;
  unsigned wordByteSize = 2; // half float, i.e., fp16 is 2-byte in size
  long long unsigned curAddress = mf->get_addr();
  memory_space *mem = NULL;
  memory_space_t mfMemSpace = mf->get_inst().space;
  // memory_space_t m_m_space = mf->get_inst().whichspace(my_addr);
  mf->get_w_inst()->decode_space(mfMemSpace, mem, curAddress);

  unsigned accessSize = mf->get_access_size();
  char * tBuffer = new char[accessSize + 1];
  if (mfMemSpace == global_space) {
    tBuffer[accessSize] = '\0';

    mem->read(curAddress, accessSize, tBuffer);
    uint16_t * packet_data_hex = new uint16_t[accessSize/wordByteSize];
    memcpy( (void*)packet_data_hex, (void*)tBuffer, accessSize );    

    curPair = g_icnt_interface->approx_Average_2(packet_data_hex, (accessSize/wordByteSize));
    
    mem->read(targetAddress, accessSize, tBuffer);
    uint16_t * packet_data_hex_target = new uint16_t[accessSize/wordByteSize];
    memcpy( (void*)packet_data_hex_target, (void*)tBuffer, accessSize );

    targetPair = g_icnt_interface->approx_Average_2(packet_data_hex_target, (accessSize/wordByteSize));

    delete []packet_data_hex;
    delete []packet_data_hex_target;

    if ( ((curPair._average > (targetPair._average * 0.9)) || ( curPair._average < (targetPair._average * 1.1) ) )
        &&  ((curPair._range > (targetPair._range * 0.9)) || (curPair._range < (targetPair._range * 1.1) ) ) ) {
      result = true;
    }
  }

  
  delete []tBuffer;

  return result;
}



unsigned GPUTrafficManager::computeManhDistance(unsigned majorDimension, unsigned node1, unsigned node2) {
  if (node1 == node2) {
    return 0;
  }
  return std::abs((int)((node1 / majorDimension) - (node2 / majorDimension))) 
          + std::abs((int)((node1 % majorDimension) - (node2 % majorDimension)));

}


unsigned GPUTrafficManager::estimateFullTripCost(unsigned majorDimension, unsigned source_node, unsigned destination_node, unsigned current_node) {
  
  unsigned fullTripCost;
  
  // if (g_icnt_interface->getDeviceID(current_node) < _n_shader) {}
  if (g_icnt_interface->getDeviceID(destination_node) < _n_shader) {
    // shader cluster
    fullTripCost = computeManhDistance(majorDimension, current_node, destination_node) 
                          + 2 * computeManhDistance(majorDimension, destination_node, source_node)
                          + g_icnt_interface->routedPacketPerDestCount[current_node][destination_node];
  } else {
    // mem partition, higher cost, multiplier = numOfNodes / numOfMemPart
    fullTripCost = computeManhDistance(majorDimension, current_node, destination_node) 
                          + 2 * computeManhDistance(majorDimension, destination_node, source_node)
                          + (_nodes / (_nodes - _n_shader) + 1) * g_icnt_interface->routedPacketPerDestCount[current_node][destination_node];
  }
  
  return fullTripCost;

}




GPUTrafficManager::GPUTrafficManager( const Configuration &config, const vector<Network *> &net, unsigned n_shader)
  :TrafficManager(config, net)
{
  // The total simulations equal to number of kernels
  _total_sims = 0;
  
  _input_queue.resize(_subnets);
  for ( int subnet = 0; subnet < _subnets; ++subnet) {
    _input_queue[subnet].resize(_nodes);


// Khoa
    // _net[subnet]->_traffic_manager = this;
////


    for ( int node = 0; node < _nodes; ++node ) {
      _input_queue[subnet][node].resize(_classes);


// Khoa
      // _net[subnet]->_routers[node]->_traffic_manager = this;
////


    }

  }

  // Khoa, 2022/07/
  //// initialization zone
  _n_shader = n_shader;
  // _entryLimit_cluster = 300;
  // _entryLimit_mP = 0;
  // _addressesPerMPEntry = 0;
  
  _entryLimit_cluster = config.GetInt("lut_limit_cluster") ;
  _entryLimit_mP = config.GetInt("lut_limit_mc") ;
  _addressesPerMPEntry = config.GetInt("address_per_entry_mc") ;
  _contentSharing_MC_serve_newAddress = config.GetInt("mc_serve_new_address_same_content") ;
  _use_approx_sharing = config.GetInt("use_approx_sharing") ;
  _use_approx_method = config.GetInt("use_approx_method") ;
  _use_address_sharing_only = config.GetInt("use_address_sharing_only") ;

  cout << "TEST :: ENTRY LIMIT CLUSTER: " << _entryLimit_cluster << "\n" ;
  cout << "TEST :: ENTRY LIMIT MP: " << _entryLimit_mP << ", " << _addressesPerMPEntry << "\n" ;

  NoCLUT_cluster.resize(_nodes); 
  // NoCLUT_cluster_s.resize(_nodes);
  NoCLUT_memPart.resize(_nodes); // actually only use (_nodes - _n_shader) = (n_mem) elements, the others are empty
  // NoCLUT_memPart_s.resize(_nodes);
  printCounter = 0;
  debugCounter = 0;
  printDeadlock = 0;

  // Khoa, 2023/03/28
  redirectedReadReq.resize(_nodes); 
  _majorDim = (unsigned)sqrt((unsigned)_nodes);
  ////
}

GPUTrafficManager::~GPUTrafficManager()
{
}

void GPUTrafficManager::Init()
{
  _time = 0;
  _sim_state = running;
  _ClearStats( );
  
}

void GPUTrafficManager::_RetireFlit( Flit *f, int dest )
{
  _deadlock_timer = 0;
  
  assert(_total_in_flight_flits[f->cl].count(f->id) > 0);
  _total_in_flight_flits[f->cl].erase(f->id);
  
  if(f->record) {
    assert(_measured_in_flight_flits[f->cl].count(f->id) > 0);
    _measured_in_flight_flits[f->cl].erase(f->id);
  }
  
  if ( f->watch ) {
    *gWatchOut << GetSimTime() << " | "
    << "node" << dest << " | "
    << "Retiring flit " << f->id
    << " (packet " << f->pid
    << ", src = " << f->src
    << ", dest = " << f->dest
    << ", hops = " << f->hops
    << ", flat = " << f->atime - f->itime
    << ")." << endl;
  }
  
  if ( f->head && ( f->dest != dest ) ) {
    ostringstream err;
    err << "Flit " << f->id << " arrived at incorrect output " << dest;
    Error( err.str( ) );
  }
  
  if((_slowest_flit[f->cl] < 0) ||
     (_flat_stats[f->cl]->Max() < (f->atime - f->itime)))
    _slowest_flit[f->cl] = f->id;
  
  _flat_stats[f->cl]->AddSample( f->atime - f->itime);
  if(_pair_stats){
    _pair_flat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - f->itime );
  }
  
  if ( f->tail ) {
    Flit * head;
    if(f->head) {
      head = f;
    } else {
      map<unsigned long long, Flit *>::iterator iter = _retired_packets[f->cl].find(f->pid);
      assert(iter != _retired_packets[f->cl].end());
      head = iter->second;
      _retired_packets[f->cl].erase(iter);
      assert(head->head);
      assert(f->pid == head->pid);
    }
    if ( f->watch ) {
      *gWatchOut << GetSimTime() << " | "
      << "node" << dest << " | "
      << "Retiring packet " << f->pid
      << " (plat = " << f->atime - head->ctime
      << ", nlat = " << f->atime - head->itime
      << ", frag = " << (f->atime - head->atime) - (f->id - head->id) // NB: In the spirit of solving problems using ugly hacks, we compute the packet length by taking advantage of the fact that the IDs of flits within a packet are contiguous.
      << ", src = " << head->src
      << ", dest = " << head->dest
      << ")." << endl;
    }
   
// GPGPUSim: Memory will handle reply, do not need this
#if 0
    //code the source of request, look carefully, its tricky ;)
    if (f->type == Flit::READ_REQUEST || f->type == Flit::WRITE_REQUEST) {
      PacketReplyInfo* rinfo = PacketReplyInfo::New();
      rinfo->source = f->src;
      rinfo->time = f->atime;
      rinfo->record = f->record;
      rinfo->type = f->type;
      _repliesPending[dest].push_back(rinfo);
    } else {
      if(f->type == Flit::READ_REPLY || f->type == Flit::WRITE_REPLY  ){
        _requestsOutstanding[dest]--;
      } else if(f->type == Flit::ANY_TYPE) {
        _requestsOutstanding[f->src]--;
      }
      
    }
#endif

    if(f->type == Flit::READ_REPLY || f->type == Flit::WRITE_REPLY  ){
      _requestsOutstanding[dest]--;
    } else if(f->type == Flit::ANY_TYPE) {
      ostringstream err;
      err << "Flit " << f->id << " cannot be ANY_TYPE" ;
      Error( err.str( ) );
    }
    
    // Only record statistics once per packet (at tail)
    // and based on the simulation state
    if ( ( _sim_state == warming_up ) || f->record ) {
      
      _hop_stats[f->cl]->AddSample( f->hops );
      
      if((_slowest_packet[f->cl] < 0) ||
         (_plat_stats[f->cl]->Max() < (f->atime - head->itime)))
        _slowest_packet[f->cl] = f->pid;
      _plat_stats[f->cl]->AddSample( f->atime - head->ctime);
      _nlat_stats[f->cl]->AddSample( f->atime - head->itime);
      _frag_stats[f->cl]->AddSample( (f->atime - head->atime) - (f->id - head->id) );
      
      if(_pair_stats){
        _pair_plat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - head->ctime );
        _pair_nlat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - head->itime );
      }
    }
    
    if(f != head) {
      head->Free();
    }
    
  }
  
  if(f->head && !f->tail) {
    _retired_packets[f->cl].insert(make_pair(f->pid, f));
  } else {
    f->Free();
  }
}
int  GPUTrafficManager::_IssuePacket( int source, int cl )
{
  return 0;
}

//TODO: Remove stype?
void GPUTrafficManager::_GeneratePacket(
  int source, 
  int stype, 
  int cl, 
  int time, 
  int subnet, 
  int packet_size, 
  const Flit::FlitType& packet_type, 
  void* const data, 
  int dest)
{
  assert(stype!=0);
  
// Khoa
mem_fetch* mf = static_cast<mem_fetch*>(data);
unsigned long long mfBlAddress = mf->get_addr();
// std::string exactDataKey;
// std::string dataKey = makeDataKey(mf, exactDataKey);

  //  Flit::FlitType packet_type = Flit::ANY_TYPE;
  int size = packet_size; //input size
  unsigned long long pid = _cur_pid++;
  assert(_cur_pid > 0);
  int packet_destination = dest;
  bool record = false;
  bool watch = gWatchOut && (_packets_to_watch.count(pid) > 0);
  


//// In GPGPUSim, the core specified the packet_type and size
#if 0
  if(_use_read_write[cl]){
    if(stype > 0) {
      if (stype == 1) {
        packet_type = Flit::READ_REQUEST;
        size = _read_request_size[cl];
      } else if (stype == 2) {
        packet_type = Flit::WRITE_REQUEST;
        size = _write_request_size[cl];
      } else {
        ostringstream err;
        err << "Invalid packet type: " << packet_type;
        Error( err.str( ) );
      }
    } else {
      PacketReplyInfo* rinfo = _repliesPending[source].front();
      if (rinfo->type == Flit::READ_REQUEST) {//read reply
        size = _read_reply_size[cl];
        packet_type = Flit::READ_REPLY;
      } else if(rinfo->type == Flit::WRITE_REQUEST) {  //write reply
        size = _write_reply_size[cl];
        packet_type = Flit::WRITE_REPLY;
      } else {
        ostringstream err;
        err << "Invalid packet type: " << rinfo->type;
        Error( err.str( ) );
      }
      packet_destination = rinfo->source;
      time = rinfo->time;
      record = rinfo->record;
      _repliesPending[source].pop_front();
      rinfo->Free();
    }
  }
#endif
////


  if ((packet_destination < 0) || (packet_destination >= _nodes)) {
    ostringstream err;
    err << "Incorrect packet destination " << packet_destination
    << " for stype " << packet_type;
    Error( err.str( ) );
  }
  
  if ( ( _sim_state == running ) ||
      ( ( _sim_state == draining ) && ( time < _drain_time ) ) ) {
    record = _measure_stats[cl];
  }
  
  int subnetwork = subnet;
  //                ((packet_type == Flit::ANY_TYPE) ?
  //                    RandomInt(_subnets-1) :
  //                    _subnet[packet_type]);
  
  if ( watch ) {
    *gWatchOut << GetSimTime() << " | "
    << "node" << source << " | "
    << "Enqueuing packet " << pid
    << " at time " << time
    << "." << endl;
  }
  
  for ( int i = 0; i < size; ++i ) {
    Flit * f  = Flit::New();
    f->id     = _cur_id++;
    assert(_cur_id);
    f->pid    = pid;
    f->watch  = watch | (gWatchOut && (_flits_to_watch.count(f->id) > 0));
    f->subnetwork = subnetwork;
    f->src    = source;
    f->ctime  = time;
    f->record = record;
    f->cl     = cl;
    f->data = data;
    
    _total_in_flight_flits[f->cl].insert(make_pair(f->id, f));
    if(record) {
      _measured_in_flight_flits[f->cl].insert(make_pair(f->id, f));
    }
    
    if(gTrace){
      cout<<"New Flit "<<f->src<<endl;
    }
    f->type = packet_type;


    // f->_traffic_manager = this; // Khoa, 2022/08/

    f->vc  = -1;
    f->redirectedFlag = false; // set first before rechecking
    f->isProbeFlag = false; // set first before rechecking

    if ( i == 0 ) { // Head flit
      /// header
      f->head = true;
      //packets are only generated to nodes smaller or equal to limit
      
      long long unsigned curTime = getTime();
      unsigned homebaseIcntID = g_icnt_interface->getHomebase(mf);
      
      mf->homebaseIcntID = homebaseIcntID; // for tracking

      f->dest = packet_destination;
      f->destMC = packet_destination; // Khoa, 2022/07/
      // f->probeDelay = 3; // unused currently

      //// reroute section
      if (packet_type == Flit::READ_REQUEST) {
        //// packet is read request

        f->destMC = packet_destination; // set MC destination for read request
        mf->mcIcntID = packet_destination; // for tracking
        f->dest = homebaseIcntID; // set f's dest first for reroutePacket(), which will be changed below
        f->isProbeFlag = true; // set probe flag to true so that source handles f as a probe // source will change it to false after finish handling

        if (source != homebaseIcntID) {
          //// read request's source is not homebase, send probe to homebase first

        } else {
          //// source == homebaseIcntID
          //// probe handled by source, 
          //// since only shaders send read request, source is a shader, not MC
          
          // f->redirectedFlag = reroutePacket(source, f); // ...f's redirected flag is actually set within that function already
          reroutePacket(source, f);
        }
        
      } else {
        //// packet is not read request

        f->dest = packet_destination;

        if (packet_type == Flit::WRITE_REQUEST) {
          //// write request -> MC
          
          f->destMC = packet_destination; // set MC destination for write request
        } else {
          //// read reply or write reply -> shader cluster

          f->destMC = -1; // set MC for responses

          if (packet_type == Flit::READ_REPLY) {
            //// read reply
            
            std::string exactDataKey;
            std::string dataKey = makeDataKey(mf, exactDataKey);
            
            
            if (mf->redirectedAddress) {
              //// in case of non-root redirect, update for homebase of root
              homebaseIcntID = g_icnt_interface->getHomebase(mf->redirectedAddress);
              NoCLUT_cluster[homebaseIcntID][mf->redirectedAddress].changeNode(packet_destination, curTime, source);
              enforceLUTEntryLimit_cluster(homebaseIcntID); 
              g_icnt_interface->totalHops_Update += computeManhDistance(_majorDim, source, homebaseIcntID);
              g_icnt_interface->updateMessageCount++; // Khoa, 2023/04/03
            } // Khoa, 2023/02/27

            if (g_icnt_interface->getDeviceID(source) < _n_shader) {
              //// source is peer cluster
              mf->peerIcntID = source; // for tracking

              //// update homebase of new address
              homebaseIcntID = g_icnt_interface->getHomebase(mfBlAddress);
              NoCLUT_cluster[homebaseIcntID][mfBlAddress].changeNode(packet_destination, curTime, source); // packet_destination is requester, source is peer cluster
              enforceLUTEntryLimit_cluster(homebaseIcntID); // Khoa, 2023/02
              g_icnt_interface->totalHops_Update += computeManhDistance(_majorDim, source, homebaseIcntID);
              g_icnt_interface->updateMessageCount++; // Khoa, 2023/04/03

            } else {
              //// source is MC
              mf->mcIcntID = source; // for tracking

              //// update homebase of new address
              homebaseIcntID = g_icnt_interface->getHomebase(mfBlAddress);
              NoCLUT_cluster[homebaseIcntID][mfBlAddress].insertSharingInfo(packet_destination, curTime, source); // packet_destination is requester, source is MC
              enforceLUTEntryLimit_cluster(homebaseIcntID); // Khoa, 2023/02
              g_icnt_interface->totalHops_Update += computeManhDistance(_majorDim, source, homebaseIcntID);
              g_icnt_interface->updateMessageCount++; // Khoa, 2023/04/03

              //// Khoa, 2023/02/05
              if ( _use_address_sharing_only == 0) {
                if (dataKey != "") {
                  if ( checkLUTEntry_memPart(source, dataKey)) {
                    //// found a previously accessed addresses with approx content
                    if (_contentSharing_MC_serve_newAddress == 0) {
                      //// 2nd design: redirect READ request to last sharer (likely, data will be approx)
                      f->type = Flit::READ_REQUEST;
                      f->subnetwork = 0;
                      subnetwork = 0;
                      f->dest = NoCLUT_memPart[source][dataKey]._lastSharerIcntID;
                      f->src = packet_destination;
                      
                      f->redirectedFlag = true;
                      f->isProbeFlag = false;

                      // //// modifying actual data in memory and reload
                      // //// only used if first access of an address is also redirected by MC, or replied with approx data
                      // //// not used when first access of an address is replied with exact data
                      // mf->redirectedAddress = NoCLUT_memPart[source][dataKey]._lastSharingAddress; // set redirect address
                      // if (_use_approx_sharing) {
                      //   g_icnt_interface->changeMfData(mf, f->src);
                      // } // 2023/03/31, moved
                      // ////
                      // Khoa, 2023/03/28
                      redirectedReadReq[source]++;
                    } else {
                      g_icnt_interface->totalReadReply_MC++;
                    }
                    
                    //// update multi roots
                    //// update homebases of previous sharing addresses
                    unordered_map< long long unsigned, long long unsigned >::iterator itrt_memPart;
                    long long unsigned sharingAddress;
                    for ( itrt_memPart = NoCLUT_memPart[source][dataKey]._sharingAddresses.begin();
                          itrt_memPart != NoCLUT_memPart[source][dataKey]._sharingAddresses.end();
                          itrt_memPart++ ) {
                      sharingAddress = itrt_memPart->first;
                      homebaseIcntID = g_icnt_interface->getHomebase(sharingAddress);

                      //// current access's  block address will be new sharing address
                      //// f's source, i.e., the requester, will be the new sharer
                      g_icnt_interface->totalHops_Update += computeManhDistance(_majorDim, source, homebaseIcntID); // source = MC IcntId
                      g_icnt_interface->updateMessageCount++; // Khoa, 2023/04/03
                      if (_contentSharing_MC_serve_newAddress == 0) { // fix update bug
                        NoCLUT_cluster[homebaseIcntID][sharingAddress].changeSharingInfo(mfBlAddress, f->src, curTime, source); // f->src = requester, source = MC IcntId
                      } else {
                        NoCLUT_cluster[homebaseIcntID][sharingAddress].changeSharingInfo(mfBlAddress, f->dest, curTime, source); // f->dest = requester, source = MC IcntId
                      }
                      enforceLUTEntryLimit_cluster(homebaseIcntID);
                      // NoCLUT_cluster[homebaseIcntID].erase(sharingAddress); // remove old entry, for keep-only-last-info-entry design
                    }
                    //// 2023/03/ , update LuT_mem
                    NoCLUT_memPart[source][dataKey].insertSharingAddress(mfBlAddress, curTime, packet_destination);
                    ////

                  } else {
                    //// entry not found, add new entry, 
                    add_LUT_entry_memPart(source, dataKey, packet_destination, mfBlAddress); // Khoa, 2023/02/05

                    // MC serves data
                    g_icnt_interface->totalReadReply_MC++;
                  } // end if (found LUT entry) else ()
                } // end if (dataKey != "")
              } // end if (use address sharing only or use content sharing)
            } // end if (source is cluster) else (source is MC)
            
          } else {
            //// write reply
            
            mf->mcIcntID = source; // for tracking
          }
        } 
      }
      //// end reroute section

      //// tracking test
      // if (mfBlAddress == 0xc0359ac0) {
      //   printLUT_test(source, f, curTime, "source");
      // }
      ////

    } else {
      //// not head
      f->head = false;
      f->dest = -1;
      f->destMC = -1; // Khoa, 2022/07/
      f->probeDelay = 0;
      f->isProbeFlag = false;
      f->redirectedFlag = false;
    } // end if(head) else

    if ( i == ( size - 1 ) ) { // Tail flit
      f->tail = true;
    } else {
      f->tail = false;
    }

    //// Khoa, 2022/07/    
    // if ( g_icnt_interface->getDeviceID(source) >= _n_shader ) {
    // // packet from memPart to cluster
    //   // f->holderNodes = NoCLUT_memPart[source][dataKey];
    //   // f->holderNodes2 = NoCLUT_cluster[source][mfBlAddress]._icntIDs;
    // } //


    switch( _pri_type ) {
      case class_based:
        f->pri = _class_priority[cl];
        assert(f->pri >= 0);
        break;
      case age_based:
        // f->pri = numeric_limits<int>::max() - mf->get_timestamp(); // read reply inherit priority of its corresponding request
        f->pri = numeric_limits<unsigned>::max() - (100 * (time + 1)) + computeManhDistance(_majorDim, f->src, f->dest); // give priority to long-distance packet
        // f->pri = numeric_limits<int>::max() - time;
        assert(f->pri >= 0);
        break;
      case sequence_based:
        f->pri = numeric_limits<int>::max() - _packet_seq_no[source];
        assert(f->pri >= 0);
        break;
      default:
        f->pri = 0;
    }

    if (f->pid == 0) {
      f->watch = true;
    }
    if ( f->watch ) {
      *gWatchOut << GetSimTime() << " | "
      << "node" << source << " | "
      << "Enqueuing flit " << f->id
      << " (packet " << f->pid
      << ") at time " << time
      << "." << endl;
    }
    
    // _input_queue[subnet][source][cl].push_back( f ); // Khoa
    _input_queue[subnetwork][source][cl].push_back( f );
  }



// Khoa, 2022/07/

// if (subnet == 1) {
//   unsigned long long int my_addr;
//   my_addr=mf->get_addr();
//   memory_space *mem = NULL;
//   memory_space_t m_m_space = mf->get_inst().space;
//   // memory_space_t m_m_space = mf->get_inst().whichspace(my_addr);
//   mf->get_w_inst()->decode_space(m_m_space, mem, my_addr);

//   unsigned my_access_size = mf->get_access_size();
//   char * tBuffer = new char[my_access_size+1];

//   if (m_m_space == global_space) {
//     // for (int idx=0; idx< size; idx+=8) {

//       mem->read(my_addr, my_access_size, tBuffer);
//       // mem->read(my_addr+idx, 8, tBuffer+idx);

//     // }
    
//   }

//   tBuffer[my_access_size] = '\0';


//   // FILE *resOutFile_ = fopen("testNoC_.csv", "a");  

//   //   fprintf(resOutFile_, "%u,noc,,,%u__,%u--%u,%u,0x%llx,%d,0x%llx,%u,%s\n", 
//   //   mf->get_w_inst()->get_uid(),
//   //   mf->get_sid(),
//   //   mf->get_wid(),
//   //   mf->get_w_inst()->warp_id(),
//   //   mf->get_w_inst()->dynamic_warp_id(),
//   //   mf->get_w_inst()->pc,
//   //   m_m_space.get_type(),
//   //   my_addr,
//   //   my_access_size,
//   //   tBuffer
//   //   );

//   // printBits("testPacket_.csv", my_access_size, tBuffer);
//   // FILE *resOutFile_1 = fopen("testPacket_.csv", "a");
//   // fprintf(resOutFile_1, "\n");
//   // fclose(resOutFile_1);

    

//   // fprintf(resOutFile_, "%u,noc,%llu,%llu,%u__,%u--%u,%u,0x%llx,%d,0x%llx,%u,%s\n", 
//   //   mf->get_w_inst()->get_uid(),
//   //   mf->get_w_inst()->m_config->gpgpu_ctx->the_gpgpusim->g_the_gpu->gpu_sim_cycle, 
//   //   mf->get_w_inst()->m_config->gpgpu_ctx->the_gpgpusim->g_the_gpu->gpu_tot_sim_cycle,
//   //   mf->get_sid(),
//   //   mf->get_wid(),
//   //   mf->get_w_inst()->warp_id(),
//   //   mf->get_w_inst()->dynamic_warp_id(),
//   //   mf->get_w_inst()->pc,
//   //   m_m_space.get_type(),
//   //   my_addr,
//   //   size,
//   //   tBuffer
//   //   );

//   // fclose(resOutFile_); //

//   delete []tBuffer;
// } ////

}



void GPUTrafficManager::_Step(long long unsigned curCycle)
{

// FILE *resOutFile_step = fopen("testNoCLUT_step_.csv", "a"); // Khoa
long long unsigned curTime = getTime();

// ////
// unsigned traceNodeID = 50;
// long long unsigned traceAddress = 0xc0321aa0;
// std::string traceKey = "0x3a003a0038003800380038003800380038003800380038003800380038003800";
// if (NoCLUT_cluster[traceNodeID].count(traceAddress)) {
//   FILE *resOutFile_traceStep1 = fopen("testNoCLUT_traceStep_cluster_.csv", "a");

//   fprintf(resOutFile_traceStep1,
//     "%llu,sharer:%d,%u,0x%08llx,%llu,%d\n",
//       getTime(),
//       NoCLUT_cluster[traceNodeID][traceAddress]._sharerIcntID,
//       NoCLUT_cluster[traceNodeID][traceAddress]._isRoot,
//       NoCLUT_cluster[traceNodeID][traceAddress]._rootAddress,
//       NoCLUT_cluster[traceNodeID][traceAddress]._lastChangeTime,
//       NoCLUT_cluster[traceNodeID][traceAddress]._lastChangerIcntID
//   );

//   fclose(resOutFile_traceStep1);
// }

// traceNodeID = 13;
// if (NoCLUT_memPart[traceNodeID].count(traceKey)) {
//   if (NoCLUT_memPart[traceNodeID][traceKey].count(traceAddress)) {
//     FILE *resOutFile_traceStep2 = fopen("testNoCLUT_traceStep_mP_.csv", "a");

//     fprintf(resOutFile_traceStep2,
//       "%llu",
//         getTime()
//     );

//     unordered_map< long long unsigned, long long unsigned >::iterator itrt_memPart;
//     for ( itrt_memPart = NoCLUT_memPart[traceNodeID][traceKey].begin();
//           itrt_memPart != NoCLUT_memPart[traceNodeID][traceKey].end();
//           itrt_memPart++ ) {
//       fprintf(resOutFile_traceStep2,
//         ",0x%08llx | %llu", 
//           itrt_memPart->first,
//           itrt_memPart->second
//       );
//     }

//     fprintf(resOutFile_traceStep2, "\n");

//     fclose(resOutFile_traceStep2);
//   }

// }

////


  bool flits_in_flight = false;
  for(int c = 0; c < _classes; ++c) {
    flits_in_flight |= !_total_in_flight_flits[c].empty();
  }
  if(flits_in_flight && (_deadlock_timer++ >= _deadlock_warn_timeout)){
    _deadlock_timer = 0;
    cout << "WARNING: Possible network deadlock (GPUTrafficManager).\n";

  //// deadlock check print out
  if (printDeadlock < 3) {
    FILE *resOutFile_d = fopen("testRedirectPackerPerDest_onDeadLock_.csv", "a");
    for (unsigned idx = 0; idx < _nodes; idx++ ) {
      if ( g_icnt_interface->getDeviceID(idx) < _n_shader ) {
        fprintf(resOutFile_d, "cluster node# %03u/%u:%u,%u\n", 
          idx,
          NoCLUT_memPart.size(), 
          _nodes,
          g_icnt_interface->redirectPacketPerDestCount[idx]
            );
      } else {
        fprintf(resOutFile_d, "mem node# %03u/%u:%u,%u\n", 
          idx, 
          NoCLUT_memPart.size(), 
          _nodes,
          g_icnt_interface->redirectPacketPerDestCount[idx]
          );
      }

    }
    fclose(resOutFile_d);

    FILE *resOutFile_ppc = fopen("testPushPopCount_onDeadLock_.csv", "a");
    fprintf(resOutFile_ppc, 
      "_,%lld,redir:%u,%u,_,push:%u,pop:%u,pop_noData:%u,", 
        getTime(),
        g_icnt_interface->redirectedCount,
        g_icnt_interface->redirectedPackets.size(),
        g_icnt_interface->pushCount,
        g_icnt_interface->popCount,
        g_icnt_interface->popCount_noData
    );
    fprintf(resOutFile_ppc, 
      "_,rReq:%u,wReq:%u,rRes:%u,wRes:%u,rReq:%u,wReq:%u,rRes:%u,wRes:%u,changed:%u,wrongSpace:%u\n", 
        g_icnt_interface->readRequestPushCount,
        g_icnt_interface->writeRequestPushCount,
        g_icnt_interface->readReplyPushCount,
        g_icnt_interface->writeReplyPushCount,
        g_icnt_interface->readRequestPopCount,
        g_icnt_interface->writeRequestPopCount,
        g_icnt_interface->readReplyPopCount,
        g_icnt_interface->writeReplyPopCount,
        g_icnt_interface->changedCount,
        g_icnt_interface->wrongSpaceCount
    );
    fclose(resOutFile_ppc);

    printDeadlock++;
  }

  }
  ////
  
  vector<map<int, Flit *> > flits(_subnets);
  
  for ( int subnet = 0; subnet < _subnets; ++subnet ) {
    for ( int n = 0; n < _nodes; ++n ) {
      
      Flit * const f = _net[subnet]->ReadFlit( n );

      if ( f ) {
        if(f->watch) {
          *gWatchOut << GetSimTime() << " | "
          << "node" << n << " | "
          << "Ejecting flit " << f->id
          << " (packet " << f->pid << ")"
          << " from VC " << f->vc
          << "." << endl;
        }

        //// Khoa, 2022/07/
        // if (f->pid == 944035) {
        //   mem_fetch* mffp = static_cast<mem_fetch* >(f->data); // Khoa
        //   unsigned requester_icntID = g_icnt_interface->getIcntID(mffp->get_tpc());
        //   unsigned memPart_icntID = f->destMC;

        //   fprintf(resOutFile_step, "%llu,%llu,%u,%03u,%03u,%03u,%03u\n",
        //     mffp->get_w_inst()->issue_cycle,
        //     curCycle,
        //     subnet,
        //     n,
        //     memPart_icntID,
        //     requester_icntID,
        //     f->dest
        //   );
          
        // }
        ////

        g_icnt_interface->WriteOutBuffer(subnet, n, f);
      }
      
      g_icnt_interface->Transfer2BoundaryBuffer(subnet, n);

      Flit* const ejected_flit = g_icnt_interface->GetEjectedFlit(subnet, n);
      
      if (ejected_flit) {
        if(ejected_flit->head)
          assert(ejected_flit->dest == n);
        if(ejected_flit->watch) {
          *gWatchOut << GetSimTime() << " | "
          << "node" << n << " | "
          << "Ejected flit " << ejected_flit->id
          << " (packet " << ejected_flit->pid
          << " VC " << ejected_flit->vc << ")"
          << "from ejection buffer." << endl;
        }
        flits[subnet].insert(make_pair(n, ejected_flit));
        if((_sim_state == warming_up) || (_sim_state == running)) {
          ++_accepted_flits[ejected_flit->cl][n];
          if(ejected_flit->tail) {
            ++_accepted_packets[ejected_flit->cl][n];
          }
        }
      }
    
      // Processing the credit From the network
      Credit * const c = _net[subnet]->ReadCredit( n );
      if ( c ) {
#ifdef TRACK_FLOWS
        for(set<int>::const_iterator iter = c->vc.begin(); iter != c->vc.end(); ++iter) {
          int const vc = *iter;
          assert(!_outstanding_classes[n][subnet][vc].empty());
          int cl = _outstanding_classes[n][subnet][vc].front();
          _outstanding_classes[n][subnet][vc].pop();
          assert(_outstanding_credits[cl][subnet][n] > 0);
          --_outstanding_credits[cl][subnet][n];
        }
#endif
        _buf_states[n][subnet]->ProcessCredit(c);
        c->Free();
      }

    } // end for(nodes)
    _net[subnet]->ReadInputs( );
  } // end for(subnets)



// GPGPUSim will generate/inject packets from interconnection interface
#if 0
  if ( !_empty_network ) {
    _Inject();
  }
#endif
  


  for(int subnet = 0; subnet < _subnets; ++subnet) {
    
    for(int n = 0; n < _nodes; ++n) {
      
      Flit * f = NULL;
      
      BufferState * const dest_buf = _buf_states[n][subnet];
      
      int const last_class = _last_class[n][subnet];
      
      int class_limit = _classes;

      if(_hold_switch_for_packet) {
        list<Flit *> const & pp = _input_queue[subnet][n][last_class];
        if(!pp.empty() && !pp.front()->head &&
           !dest_buf->IsFullFor(pp.front()->vc)) {
          f = pp.front();
          assert(f->vc == _last_vc[n][subnet][last_class]);
          
          // if we're holding the connection, we don't need to check that class
          // again in the for loop
          --class_limit;
        }
      }
      
      for(int i = 1; i <= class_limit; ++i) {
        
        int const c = (last_class + i) % _classes;
        
        list<Flit *> const & pp = _input_queue[subnet][n][c];
        
        if(pp.empty()) {
          continue;
        }
        
        Flit * const cf = pp.front();
        assert(cf);
        assert(cf->cl == c);
        assert(cf->subnetwork == subnet);
        
        if(f && (f->pri >= cf->pri)) {
          continue;
        }
        
        //// Khoa, 2022/07/
        // if (cf->pid == 944035) {
        // mem_fetch* mffp2 = static_cast<mem_fetch* >(cf->data); // Khoa
        // unsigned requester_icntID = g_icnt_interface->getIcntID(mffp2->get_tpc());
        // unsigned memPart_icntID = cf->destMC;

        // fprintf(resOutFile_step, "%llu,%llu,%u,%03u,%03u,%03u,%03u\n",
        //   mffp2->get_w_inst()->issue_cycle,
        //   curCycle,
        //   subnet,
        //   n,
        //   memPart_icntID,
        //   requester_icntID,
        //   cf->dest
        //   );
        // }
        ////

        if(cf->head) {
          mem_fetch* mft = static_cast<mem_fetch* >(cf->data);
          // unsigned long long blAddress = mft->get_addr();
          unsigned long long blockAddress = mft->get_addr();
          std::string exactDataKey;
          std::string dataKey = makeDataKey(mft, exactDataKey);
          
          if (dataKey != "") {
            if(subnet == 0) {
              // request network
              // check destination
              // change destination to nearer data holder if necessary
              if ( cf->type == Flit::READ_REQUEST ) {
              }
            } else {
              // subnet == 1, response network
              if (cf->type == Flit::READ_REPLY) {
                //// read reply
                
                //// update LUT_cluster at new homebase and LUT_mP at current MC (opportunistic way)
                //// insert new entry at new address's homebase
                unsigned homebaseIcntID = g_icnt_interface->getHomebase(blockAddress);
                long long unsigned sharingAddress;

                //// update homebase of new address
                //// moved to generate_packet()
                // g_icnt_interface->totalHops_Update += computeManhDistance(8, n, homebaseIcntID);
                // NoCLUT_cluster[homebaseIcntID][blockAddress].insertSharingInfo(cf->dest, curTime, n); // n = cf->src
                // enforceLUTEntryLimit_cluster(homebaseIcntID); // Khoa, 2023/02/05

                if (_use_address_sharing_only == 0) {
                  if (g_icnt_interface->getDeviceID(cf->src) >= _n_shader) {
                    //// from MC 
                    //// update LUT of memPart 
                    //// for update if same content set accessed again later (same procedure, but different purpose from MC/centralized design)
                    
                    //// update LUT_cluster for entries on LUT_mP
                    // if ( checkLUTEntry_memPart(cf->src, dataKey) ) {
                    //   /// found a previously accessed addresses with approx content

                    //   //// modifying actual data in memory and reload
                    //   //// only used if first access of an address is also redirected by MC, or replied with approx data
                    //   //// not used when first access of an address is replied with exact data
                    //   // mft->redirectedAddress = NoCLUT_memPart[cf->src][dataKey]._lastSharingAddress; // set redirect address
                    //   // g_icnt_interface->changeMfData(mft, cf->dest); 
                    //   ////

                    //   //// memory controller handling reply (with exact data)

                    //   //// update multi roots
                    //   //// update homebases of previous sharing addresses
                    //   unordered_map< long long unsigned, long long unsigned >::iterator itrt_memPart;
                    //   for ( itrt_memPart = NoCLUT_memPart[cf->src][dataKey]._sharingAddresses.begin();
                    //         itrt_memPart != NoCLUT_memPart[cf->src][dataKey]._sharingAddresses.end();
                    //         itrt_memPart++ ) {
                    //     sharingAddress = itrt_memPart->first;
                    //     homebaseIcntID = g_icnt_interface->getHomebase(sharingAddress);

                    //     //// current access's  block address will be new sharing address
                    //     //// f's source, i.e., the requester, will be the new sharer
                    //     g_icnt_interface->totalHops_Update += computeManhDistance(8, cf->src, homebaseIcntID);
                    //     NoCLUT_cluster[homebaseIcntID][sharingAddress].changeSharingInfo(blockAddress, cf->dest, curTime, cf->src);
                    //     enforceLUTEntryLimit_cluster(homebaseIcntID);
                    //     // NoCLUT_cluster[homebaseIcntID].erase(sharingAddress); // remove old entry, for keep-only-last-info-entry design
                    //   }
                    // }
                    ////

                    
                    // add_LUT_entry_memPart(cf->src, dataKey, cf->dest, blockAddress); // Khoa, 2023/02/05
                    // add_LUT_entry_memPart(n, dataKey, cf->dest, blockAddress);
                    ////
                  } 
                }   
              }
            }
          }
        } //// end if (cf->head)



        if(cf->head && cf->vc == -1) { 
          //// Find first available VC
          
          OutputSet route_set;

          // if (cf->pid == 141592) {
          //   FILE *resOutFile_rf1 = fopen("testRF_1_.csv", "a");
          //   fprintf(resOutFile_rf1, "%03u,,%03u | %03u\n", 
          //     n,
          //     cf->destMC, 
          //     cf->dest
          //     );
          //   fclose(resOutFile_rf1);
          // }

          _rf(NULL, cf, -1, &route_set, true);
          set<OutputSet::sSetElement> const & os = route_set.GetSet();
          assert(os.size() == 1);
          OutputSet::sSetElement const & se = *os.begin();
          assert(se.output_port == -1);
          int vc_start = se.vc_start;
          int vc_end = se.vc_end;
          int vc_count = vc_end - vc_start + 1;
          if(_noq) {
            assert(_lookahead_routing);
            const FlitChannel * inject = _net[subnet]->GetInject(n);
            const Router * router = inject->GetSink();
            assert(router);
            int in_channel = inject->GetSinkPort();
            
            // NOTE: Because the lookahead is not for injection, but for the
            // first hop, we have to temporarily set cf's VC to be non-negative
            // in order to avoid seting of an assertion in the routing function.
            cf->vc = vc_start;

            // if (cf->pid == 141592) {
            //   FILE *resOutFile_rf2 = fopen("testRF_2_.csv", "a");
            //   fprintf(resOutFile_rf2, "%03u,%03u,%03u | %03u\n", 
            //     n,
            //     router->GetID(),
            //     cf->destMC, 
            //     cf->dest
            //     );
            //   fclose(resOutFile_rf2);
            // }

            _rf(router, cf, in_channel, &cf->la_route_set, false);
            cf->vc = -1;
            
            if(cf->watch) {
              *gWatchOut << GetSimTime() << " | "
              << "node" << n << " | "
              << "Generating lookahead routing info for flit " << cf->id
              << " (NOQ)." << endl;
            }
            set<OutputSet::sSetElement> const sl = cf->la_route_set.GetSet();
            assert(sl.size() == 1);
            int next_output = sl.begin()->output_port;
            vc_count /= router->NumOutputs();
            vc_start += next_output * vc_count;
            vc_end = vc_start + vc_count - 1;
            assert(vc_start >= se.vc_start && vc_start <= se.vc_end);
            assert(vc_end >= se.vc_start && vc_end <= se.vc_end);
            assert(vc_start <= vc_end);
          } // end if(noq)

          if(cf->watch) {
            *gWatchOut << GetSimTime() << " | " << FullName() << " | "
            << "Finding output VC for flit " << cf->id
            << ":" << endl;
          }

          for(int i = 1; i <= vc_count; ++i) {
            int const lvc = _last_vc[n][subnet][c];
            int const vc =
              (lvc < vc_start || lvc > vc_end) ?
                vc_start :
                (vc_start + (lvc - vc_start + i) % vc_count);
            assert((vc >= vc_start) && (vc <= vc_end));

            if(!dest_buf->IsAvailableFor(vc)) {
              if(cf->watch) {
                *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                << "  Output VC " << vc << " is busy." << endl;
              }
            } else {
              if(dest_buf->IsFullFor(vc)) {
                if(cf->watch) {
                  *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                  << "  Output VC " << vc << " is full." << endl;
                }
              } else {
                if(cf->watch) {
                  *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                  << "  Selected output VC " << vc << "." << endl;
                }
                cf->vc = vc;
                break;
              }
            }
          }
        }
        
        if(cf->vc == -1) {
          if(cf->watch) {
            *gWatchOut << GetSimTime() << " | " << FullName() << " | "
            << "No output VC found for flit " << cf->id
            << "." << endl;
          }
        } else {
          if(dest_buf->IsFullFor(cf->vc)) {
            if(cf->watch) {
              *gWatchOut << GetSimTime() << " | " << FullName() << " | "
              << "Selected output VC " << cf->vc
              << " is full for flit " << cf->id
              << "." << endl;
            }
          } else {
            f = cf;
          }
        }
      }
      

      if(f) {
        
        assert(f->subnetwork == subnet);
        
        // bool hasVCBuffer;
        // if (f->isProbeFlag) {
        //   hasVCBuffer = g_icnt_interface->HasVCBuffer(g_icnt_interface->getDeviceID(n), 1, 0, 1);
        // } else {
        //   hasVCBuffer = true;
        // }
        // if (hasVCBuffer) {
          int const c = f->cl;

          //// Khoa
          // if (f->pid == 944035) {
          // mem_fetch* mffp3 = static_cast<mem_fetch* >(f->data); // Khoa
          // unsigned requester_icntID = g_icnt_interface->getIcntID(mffp3->get_tpc());
          // unsigned memPart_icntID = f->destMC;

          // fprintf(resOutFile_step, "%llu,%llu,%u,%03u,%03u,%03u,%03u\n",
          //   mffp3->get_w_inst()->issue_cycle,
          //   curCycle,
          //   subnet,
          //   n,
          //   memPart_icntID,
          //   requester_icntID,
          //   f->dest
          //   );
          // }
          ////


          if(f->head) {

            if (_lookahead_routing) {
              if(!_noq) {
                const FlitChannel * inject = _net[subnet]->GetInject(n);
                const Router * router = inject->GetSink();
                assert(router);
                int in_channel = inject->GetSinkPort();

                // if (f->pid == 141592) {
                //   FILE *resOutFile_rf3 = fopen("testRF_3_.csv", "a");
                //   fprintf(resOutFile_rf3, "%03u,%03u,%03u | %03u\n", 
                //     n,
                //     router->GetID(),
                //     f->destMC, 
                //     f->dest
                //     );
                //   fclose(resOutFile_rf3);
                // }

                _rf(router, f, in_channel, &f->la_route_set, false);

                if(f->watch) {
                  *gWatchOut << GetSimTime() << " | "
                  << "node" << n << " | "
                  << "Generating lookahead routing info for flit " << f->id
                  << "." << endl;
                }
              } else if(f->watch) {
                *gWatchOut << GetSimTime() << " | "
                << "node" << n << " | "
                << "Already generated lookahead routing info for flit " << f->id
                << " (NOQ)." << endl;
              } // end if(noq) else
            } else {
              f->la_route_set.Clear();
            } // end if (look_ahead_routing) else
            
            dest_buf->TakeBuffer(f->vc);
            _last_vc[n][subnet][c] = f->vc;
          } // end if (f->head)
          
          _last_class[n][subnet] = c;
          
          _input_queue[subnet][n][c].pop_front();
          
  #ifdef TRACK_FLOWS
          ++_outstanding_credits[c][subnet][n];
          _outstanding_classes[n][subnet][f->vc].push(c);
  #endif
          
          dest_buf->SendingFlit(f);
          
          if(_pri_type == network_age_based) {
            f->pri = numeric_limits<int>::max() - _time;
            assert(f->pri >= 0);
          }
          
          if(f->watch) {
            *gWatchOut << GetSimTime() << " | "
            << "node" << n << " | "
            << "Injecting flit " << f->id
            << " into subnet " << subnet
            << " at time " << _time
            << " with priority " << f->pri
            << "." << endl;
          }
          f->itime = _time;
          
          // Pass VC "back"
          if(!_input_queue[subnet][n][c].empty() && !f->tail) {
            Flit * const nf = _input_queue[subnet][n][c].front();
            nf->vc = f->vc;
          }
          
          if((_sim_state == warming_up) || (_sim_state == running)) {
            ++_sent_flits[c][n];
            if(f->head) {
              ++_sent_packets[c][n];
            }
          }
          
  #ifdef TRACK_FLOWS
          ++_injected_flits[c][n];
  #endif
          
          _net[subnet]->WriteFlit(f, n);
        // } // end if (hasVCBuffer)
      } // end if(f)
    } // end for(nodes)
  } // end for (subnets)

  // Send the credit To the network
  for(int subnet = 0; subnet < _subnets; ++subnet) {
    for(int n = 0; n < _nodes; ++n) {
      map<int, Flit *>::const_iterator iter = flits[subnet].find(n);
      if(iter != flits[subnet].end()) {
        Flit * const f = iter->second;

        f->atime = _time;
        if(f->watch) {
          *gWatchOut << GetSimTime() << " | "
          << "node" << n << " | "
          << "Injecting credit for VC " << f->vc
          << " into subnet " << subnet
          << "." << endl;
        }
        Credit * const c = Credit::New();
        c->vc.insert(f->vc);
        _net[subnet]->WriteCredit(c, n);
        
#ifdef TRACK_FLOWS
        ++_ejected_flits[f->cl][n];
#endif
        
        _RetireFlit(f, n);
      }
    }
    flits[subnet].clear();
    // _InteralStep here
    _net[subnet]->Evaluate( );
    _net[subnet]->WriteOutputs( );
  }
  
  ++_time;
  assert(_time);
  if(gTrace){
    cout<<"TIME "<<_time<<endl;
  }
  


// fclose(resOutFile_step); ////
// printFirst = true;
}


long long unsigned LUTRecord_mP::enforceSharingLimit(long long unsigned curTime) {
    long long unsigned oldestAddress = 0;

    if (g_icnt_interface->_traffic_manager->_addressesPerMPEntry && (_sharingAddresses.size() > g_icnt_interface->_traffic_manager->_addressesPerMPEntry)) {
      unordered_map < long long unsigned, long long unsigned >::iterator itrt;
      long long unsigned oldestTime = curTime;
      
      for ( itrt = _sharingAddresses.begin();
            itrt != _sharingAddresses.end();
            itrt++ ) {
        if (itrt->second < oldestTime) {
          oldestTime = itrt->second;
          oldestAddress = itrt->first;
        }
      }

      if (oldestAddress) {
        unsigned homebaseIcntID = g_icnt_interface->getHomebase(oldestAddress);
        g_icnt_interface->_traffic_manager->removeLUTEntry_cluster(homebaseIcntID, oldestAddress);
        _sharingAddresses.erase(oldestAddress);
        // g_icnt_interface->totalLUTWrite_MC++; // Khoa, 2023/03/28, unnecessary, already ++ in chain of operations
      }
    }
    // g_icnt_interface->totalLUTRead_MC++; // Khoa, 2023/03/28, unnecessary, already ++ in chain of operations

    return oldestAddress;
  }