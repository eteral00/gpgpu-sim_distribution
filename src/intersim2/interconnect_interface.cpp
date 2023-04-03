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

#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <utility>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <set>
#include <iterator>
#include <cstring>
#include <sstream>
#include <time.h>
#include <bitset>

#include "interconnect_interface.hpp"
#include "booksim.hpp"
#include "intersim_config.hpp"
#include "network.hpp"
#include "trafficmanager.hpp"
#include "gputrafficmanager.hpp"
#include "routefunc.hpp"
#include "globals.hpp"
#include "flit.hpp"

#include "power_module.hpp"
// #include "./power/power_module.hpp"
#include "trace.h"
// #include "../trace.h"
// #include "mem_fetch.h"
// #include "../gpgpu-sim/mem_fetch.h"

// Khoa
// #include "../../libcuda/gpgpu_context.h"
// #include "../abstract_hardware_model.h"
#include "../cuda-sim/memory.h"


// Khoa, 2022/06
void printBits(std::string filePath, size_t const size, void const * const ptr)
{
    FILE *resOutFile_ = fopen(filePath.c_str(), "a");  
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;
    
    for (i = size-1; i >= 0; i--) {
        fprintf(resOutFile_, " ");
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            fprintf(resOutFile_, "%u", byte);
        }
        
        if ( (i % 2) == 0) {
          fprintf(resOutFile_, ", ");
        }
    }
    fprintf(resOutFile_, ",");
    // fprintf(resOutFile_, "\n");
    // puts("");
    fclose(resOutFile_);
}

std::string InterconnectInterface::toBitString(void const * const pointer, unsigned sizeInByte = 2) {
  unsigned char * bytePointer = (unsigned char*) pointer;
  unsigned char byte;
  unsigned bitCounter = 0;
  std::string resultStr = "";
  for (int idx1 = sizeInByte-1; idx1 >=0; idx1--) {
    for (int idx2 = 7; idx2 >=0; idx2--) {
      if ( sizeInByte == 2 && (bitCounter == 1 || bitCounter == 6) ) {
        resultStr += '|';
      } else if ( sizeInByte == 4 && (bitCounter == 1 || bitCounter == 9) ) {
        resultStr += '|';
      }
      byte = (bytePointer[idx1] >> idx2) & 1;
      resultStr += to_string(byte);
      
      bitCounter++;
    }
  }


  return resultStr;
}


void InterconnectInterface::printMfData(mem_fetch *mf, std::string filePath) {
  // long long unsigned int mfAddress = mf->get_addr();
  // long long unsigned mfRedirectedAddress = mf->redirectedAddress;
  // memory_space *mem = NULL;
  // memory_space_t mfMemSpace = mf->get_inst().space;
  // // memory_space_t mfMemspace = mf->get_inst().whichspace(mfAddress);
  // mf->get_w_inst()->decode_space(mfMemSpace, mem, mfAddress);

  // unsigned mfAccessSize = mf->get_access_size(); // size in Bytes
  // char * tBuffer = new char[mfAccessSize + 1];
  // // memset((void*)tBuffer, 0, mfAccessSize);
  

  // uint16_t * packetData_uint16 = new uint16_t[mfAccessSize/2];
  // float * packetData_float = new float[mfAccessSize/2]; // to store the converted values from half
  // vector< half_float::half > packetData_half;
  // packetData_half.resize(mfAccessSize/2);

  // // uint16_t * packetData_uint16_approx = new uint16_t[mfAccessSize/2];
  // float * packetData_float_approx = new float[mfAccessSize/2]; // to store the converted values from half
  // vector< half_float::half > packetData_half_approx;
  // packetData_half_approx.resize(mfAccessSize/2);

  // if (mfMemSpace == global_space || mfMemSpace == local_space) {
  //   mem->read(mfAddress, mfAccessSize, tBuffer);
  // }
  // tBuffer[mfAccessSize] = '\0';
  // memcpy( (void*)packetData_uint16, (void*)tBuffer, mfAccessSize );


  // std::string valBitString;
  // std::string valBitString_approx;

  // // FILE *resOutFile_mfData = fopen(filePath.c_str(), "a");
  // // fprintf(resOutFile_mfData, 
  // //   "%llu,%llu,op_%u,0x%08llx\n", 
  // //   _traffic_manager->getTime(),
  // //   mf->get_w_inst()->get_uid(),
  // //   mf->get_w_inst()->op,
  // //   mfAddress
  // //   );

  // // for (unsigned idx = 0; idx < (mfAccessSize/2); idx++) {
    
  // //   packetData_half[idx].data_ = packetData_uint16[idx];
  // //   packetData_float[idx] = float(packetData_half[idx]);
  // //   valBitString = toBitString( (void*)(&packetData_uint16[idx]), 2);

  // //   // packetData_uint16_approx[idx] = approx_Bit_16(packetData_uint16[idx], 0xFC00);
  // //   packetData_half_approx[idx].data_ = approx_Bit_16(packetData_uint16[idx], 0xFC00);
  // //   packetData_float_approx[idx] = float(packetData_half_approx[idx]);
  // //   valBitString_approx = toBitString( (void*)(&packetData_half_approx[idx].data_), 2 );

  // //   fprintf(resOutFile_mfData, ",%3f_%s,%3f_%s\n", 
  // //     packetData_float_approx[idx],
  // //     valBitString_approx.c_str(),
  // //     packetData_float[idx],
  // //     valBitString.c_str()
  // //   );
  // // }

  // // fclose(resOutFile_mfData);


  // std::stringstream mSs;
  // time_t mTimeStampt = time(NULL);
  // mSs << mTimeStampt;
  // // std::string strTimeStampt = mSs.str(); 
  // FILE *resOutFile_time = fopen("testTimeTrace_.csv", "a");
  // fprintf(resOutFile_time, 
  //   "%s,%llu,0x%08llx,icnt_rf_%d,%llu,_,_,0x%08llx,_,_,%d,%u\n", 
  //   mSs.str().c_str(),
  //   mf->get_w_inst()->get_uid(),
  //   mf->get_w_inst()->pc,
  //   mf->get_w_inst()->op,
  //   _traffic_manager->getTime(),
  //   mfAddress,
  //   mfMemSpace,
  //   mfAccessSize
  // );
  // fclose(resOutFile_time);


  // delete []packetData_float_approx;
  // delete []packetData_float;
  // delete []packetData_uint16;
  // delete []tBuffer;

}


void InterconnectInterface::changeMfData(mem_fetch *mf, unsigned srcIcntID) {
  long long unsigned int mfAddress = mf->get_addr();
  long long unsigned mfRedirectedAddress = mf->redirectedAddress;
  memory_space *mem = NULL;
  memory_space_t mfMemSpace = mf->get_inst().space;
  // memory_space_t mfMemspace = mf->get_inst().whichspace(mfAddress);
  mf->get_w_inst()->decode_space(mfMemSpace, mem, mfAddress);

  unsigned mfAccessSize = mf->get_access_size(); // size in Bytes
  char * tBuffer = new char[mfAccessSize + 1];
  // memset((void*)tBuffer, 0, mfAccessSize);
  uint16_t * packetData_uint16 = new uint16_t[mfAccessSize/2];
  uint16_t * newPacketData_uint16 = new uint16_t[mfAccessSize/2];

  if (mfMemSpace == global_space || mfMemSpace == local_space) {
    mem->read(mfAddress, mfAccessSize, tBuffer);
    tBuffer[mfAccessSize] = '\0';
    memcpy( (void*)packetData_uint16, (void*)tBuffer, mfAccessSize );
    // memset((void*)tBuffer, 0x77, mfAccessSize);
    mem->read(mfRedirectedAddress,mfAccessSize,(void*)tBuffer);
    mem->write_approx(mfAddress, mfAccessSize, (const void*)tBuffer);
    mem->read(mfAddress, mfAccessSize, (void*)tBuffer);

    memcpy( (void*)newPacketData_uint16, (void*)tBuffer, mfAccessSize );
    
    ////
    // FILE *resOutFile_mfData = fopen("testChangeMemData_rf_.csv", "a");
    // fprintf(resOutFile_mfData, 
    //   "%llu,0x%08llx,%u||,%llu,_,%llu,_%u,0x%08llx,%u,0x%08llx,", 
    //   mf->get_w_inst()->get_uid(),
    //   mf->get_w_inst()->pc,
    //   mf->get_w_inst()->op,
    //   mf->get_w_inst()->issue_cycle,
    //   _traffic_manager->getTime(),
    //   getDeviceID(srcIcntID),
    //   mfAddress,
    //   mfAccessSize,
    //   mfRedirectedAddress
    //   );

    // for (unsigned idx = 0; idx < (mfAccessSize/2); idx++) {
    //   fprintf(resOutFile_mfData, " %04x", 
    //     packetData_uint16[idx]
    //   );
    // }
    
    // fprintf(resOutFile_mfData, ",");

    // for (unsigned idx = 0; idx < (mfAccessSize/2); idx++) {
    //   fprintf(resOutFile_mfData, " %04x", 
    //     newPacketData_uint16[idx]
    //   );
    // }
    
    // fprintf(resOutFile_mfData, "\n");

    // fclose(resOutFile_mfData);
    ////
  }

  // std::stringstream mSs;
  // time_t mTimeStampt = time(NULL);
  // mSs << mTimeStampt;
  // // std::string strTimeStampt = mSs.str(); 
  // FILE *resOutFile_time = fopen("testTimeTrace_.csv", "a");
  // fprintf(resOutFile_time, 
  //   "%s,%llu,0x%08llx,icnt_rf_%d,%llu,_,_,0x%08llx,_,_,%d,%u\n", 
  //   mSs.str().c_str(),
  //   mf->get_w_inst()->get_uid(),
  //   mf->get_w_inst()->pc,
  //   mf->get_w_inst()->op,
  //   _traffic_manager->getTime(),
  //   mfAddress,
  //   mfMemSpace,
  //   mfAccessSize
  // );
  // fclose(resOutFile_time);


  delete []newPacketData_uint16;
  delete []packetData_uint16;
  delete []tBuffer;

  changedCount++;

}


AverageRangePair InterconnectInterface::approx_Average_2(uint16_t * valueBlock, unsigned numberOfVal) {
  half_float::half tempVal;
  tempVal.data_ = valueBlock[0]; // interpreted as fp16
  float tempValFloat = float(tempVal);
  float valSum = tempValFloat;
  float valMin = tempValFloat;
  float valMax = tempValFloat;
  float valAverage = 0;
  float valRange = 0;

  for (unsigned idx = 1; idx < numberOfVal; idx++) {
    tempVal.data_ = valueBlock[idx];
    tempValFloat = float(tempVal);
    valSum += tempValFloat;
    if (tempValFloat < valMin) {
      valMin = tempValFloat;
    } else if (tempValFloat > valMax) {
      valMax = tempValFloat;
    }
  }
  valRange = valMax - valMin;
  valAverage = valSum / numberOfVal;
  
  AverageRangePair approxResult;
  approxResult._average = valAverage;
  approxResult._range = valRange;
  return approxResult;
}


std::string InterconnectInterface::approx_Average(uint16_t * valueBlock, unsigned numberOfVal) {
  std::ostringstream approxResult;
  // std::ostringstream strStream;
  
  //// as UINT16 
  // long unsigned valSum = valueBlock[0];
  // uint16_t valMin = valueBlock[0];
  // uint16_t valMax = valueBlock[0];
  // uint16_t valRange = 0;

  // for (unsigned idx = 1; idx < numberOfVal; idx++) {
  //   valSum += valueBlock[idx];
  //   if (valueBlock[idx] < valMin) {
  //     valMin = valueBlock[idx];
  //   } else if (valueBlock[idx] > valMax) {
  //     valMax = valueBlock[idx];
  //   }
  // }
  // valRange = valMax - valMin;
  // approxResult << std::setfill('0') <<  std::hex << std::setw(8) << valSum << valRange ;
////

  //// as FP16 (actually converted to Float)
  half_float::half tempVal;
  tempVal.data_ = valueBlock[0];
  // float tempValFloat = float(tempVal);
  // float valSum = tempValFloat;
  // float valMin = tempValFloat;
  // float valMax = tempValFloat;
  // float valRange = 0;
  half_float::half valSum = tempVal;
  half_float::half valMin = tempVal;
  half_float::half valMax = tempVal;
  half_float::half valRange;
  half_float::half valAverage;

  for (unsigned idx = 1; idx < numberOfVal; idx++) {
    // tempVal.data_ = valueBlock[idx];
    tempVal.data_ = valueBlock[idx] & 0xff80; // Khoa, 2023/02/20, take only 9 bits out of 16 bits
    // tempValFloat = float(tempVal);
    // valSum += tempValFloat;
    // if (tempValFloat < valMin) {
    //   valMin = tempValFloat;
    // } else if (tempValFloat > valMax) {
    //   valMax = tempValFloat;
    // }
    valSum += tempVal;
    if (tempVal < valMin) {
      valMin = tempVal;
    } else if (tempVal > valMax) {
      valMax = tempVal;
    }
  }
  valRange = valMax - valMin;
  // rounded to int
  // approxResult << std::setfill('0') <<  std::hex << std::setw(8) << (int)(valSum/numberOfVal) << (unsigned)valRange ;
  // not rounding
  // approxResult << std::setfill('0') <<  std::hex << std::setw(8) << (valSum/numberOfVal) << valRange ; // Khoa, 2023/02/17
  
  // union {
  //   float f;
  //   uint32_t u;
  // } valAverage_u32 = { .f = (valSum/numberOfVal) };
  // union {
  //   float f;
  //   uint32_t u;
  // } valRange_u32 = { .f = valRange };

  // approxResult << std::setfill('0') <<  std::hex << std::setw(8) << valAverage_u32.u << "_" << valRange_u32.u;
  valAverage = valSum / numberOfVal;
  approxResult << std::setfill('0') <<  std::hex << std::setw(4) << valAverage.data_ << "," << valRange.data_ ; // << "," << valMax.data_; // Khoa, 2023/02/20 

////
  
  return approxResult.str();
}

std::string InterconnectInterface::approx_AverageBinary(uint16_t * valueBlock, unsigned numberOfVal) {
  std::ostringstream approxResult;
  
  //// as FP16 (actually converted to FP32)
  half_float::half tempVal;
  tempVal.data_ = valueBlock[0];
  float tempValFloat = float(tempVal);
  // float valSum = tempValFloat;
  // float valMin = tempValFloat;
  // float valMax = tempValFloat;
  // float valRange = 0;
  // float valAverage;
  half_float::half valSum = tempVal;
  half_float::half valMin = tempVal;
  half_float::half valMax = tempVal;
  half_float::half valRange;
  half_float::half valAverage;

  for (unsigned idx = 1; idx < numberOfVal; idx++) {
    // tempVal.data_ = valueBlock[idx];
    tempVal.data_ = valueBlock[idx] & 0xff00; // Khoa, 2023/02/22, take 8 bits out of 16 bits
    // tempValFloat = float(tempVal);
    // valSum += tempValFloat;
    // if (tempValFloat < valMin) {
    //   valMin = tempValFloat;
    // } else if (tempValFloat > valMax) {
    //   valMax = tempValFloat;
    // }
    valSum += tempVal;
    if (tempVal < valMin) {
      valMin = tempVal;
    } else if (tempVal > valMax) {
      valMax = tempVal;
    }
  }
  valRange = valMax - valMin;
  
  valAverage = (valSum / numberOfVal);

  bitset<64> binarizedBlock;
  for (unsigned idx = 0; idx < numberOfVal; idx++) {
    tempVal.data_ = valueBlock[idx];
    // tempValFloat = float(tempVal);
    // if (tempValFloat > valAverage) {
    if (tempVal > valAverage) {
      binarizedBlock[idx] = true;
    } else {
      binarizedBlock[idx] = false;
    }
  }

  
  // approxResult << std::setfill('0') <<  std::hex << std::setw(8) << valAverage << "_" << valRange << "_" << static_cast<uint32_t>(binarizedBlock.to_ullong()) ; // Khoa, 2023/02/17

  // union {
  //   float f;
  //   uint32_t u;
  // } valAverage_u32 = { .f = valAverage };
  // union {
  //   float f;
  //   uint32_t u;
  // } valRange_u32 = { .f = valRange };

  // approxResult << std::setfill('0') <<  std::hex << std::setw(8) << valAverage_u32.u << "_" << valRange_u32.u << "_" << static_cast<uint32_t>(binarizedBlock.to_ullong()) ;

  // valAverage = valSum / numberOfVal;
  // Khoa, 2022/02/22
  // valAverage.data_ = ((valAverage.data_ >> 10 ) << 10); // clear least significant 7 bits, i.e., takes only 9 bits
  // valRange.data_ = ((valRange.data_ >> 10) << 10);
  approxResult << valAverage << "," << valRange << "," << std::setfill('0') <<  std::hex << std::setw(4) << "0x" << static_cast<uint32_t>(binarizedBlock.to_ullong()) ; // Khoa, 2023/02/20 
  // approxResult << std::setfill('0') <<  std::hex << std::setw(4) << valAverage.data_ << "," << valRange.data_ << "," << static_cast<uint32_t>(binarizedBlock.to_ullong()) ; // Khoa, 2023/02/20 
  
  return approxResult.str();
}


// uint16_t InterconnectInterface::approx_Bit_general(uint16_t value, uint16_t bitMask = 0xFFFF) {
//   // wrapper function, to call the desired approximating function
//   return approx_Bit_16(value, bitMask); 
// }

uint16_t InterconnectInterface::approx_Bit_16(uint16_t value, uint16_t bitMask = 0xFFFF) {
  return (value & bitMask); 
}


uint16_t InterconnectInterface::approx_Bit_16_2Steps(uint16_t value, unsigned expLimit, unsigned numberOfMantisBit) {
  uint16_t expo;
  uint16_t approxResult;
  expo = value << 1; // remove sign-bit
  expo = expo >> 11;
  if (expo > 15 /*expLimit*/) {
    // approxResult = value >> (10 - numberOfMantisBit);
    /// shift right to remove LSBs 
    /// then shift back in case result need to be interpreted as float
    // approxResult = approxResult << (10 - numberOfMantisBit);
    approxResult = value >> (7);
    approxResult = approxResult << (7); 
  } else if (expo > 14) {
    approxResult = value >> (8);
    approxResult = approxResult << (8);
  } else if (expo > 13) {
    approxResult = value >> (9);
    approxResult = approxResult << (9);
  } else {
    approxResult = value >> (10);
    approxResult = approxResult << (10);
  }

  return approxResult; 
}

void InterconnectInterface::approx_Bit(void* value, unsigned wordByteSize = 2, unsigned bitMask = 0xFFFF) {
  if (wordByteSize == 2) {
    // uint16_t newVal = (*(uint16_t*)value) & (uint16_t)bitMask;
    // (*(uint16_t*)value) = newVal;
    // short-form for previous 2 lines, kind of hard to read
    (*(uint16_t*)value) &= (uint16_t)bitMask; 
  }
}


unsigned InterconnectInterface::getHomebase(long long unsigned address) {
  unsigned addressTag;

  ////
  // addressTag = address & 0x1F8000; // take 6 bits before the 15 least significant bits
  // addressTag = addressTag >> 15; // shift right 15 bits
  // return addressTag; // including MCs as Homebases
  // return getIcntID(addressTag % 56); // not including MCs as Homebases
  ////

  ////
  // unsigned idx0 = (address >> 8) & 0x3; // shift right 8 bits, then take 2 bits
  // unsigned idx1 = (address >> 10) & 0x3; // shift right 10 bits then take 2 bits
  // unsigned idx2 = (address >> 12) & 0x3; // shift right 12 bits then take 2 bits
  // addressTag = homebaseMap[idx0][idx1][idx2];
  // return addressTag; // including MCs as Homebases
  addressTag = address >> 8; // shift right 8 bits
  unsigned rowIdx = (addressTag & 0x3F) % _traffic_manager->_majorDim;
  addressTag = address >> 14;
  unsigned colIdx = (addressTag & 0x3F) % _traffic_manager->_majorDim;
  addressTag = (rowIdx * _traffic_manager->_majorDim) + colIdx;
  return addressTag; // including MCs as Homebases
  ////

  ////
  // addressTag = address >> 8; // shift right 8 bits
  // if (addressTag & 0x40) {
  //   addressTag = addressTag & 0x3F; // take the least significant 6 bits  
  // } else {
  //   addressTag = 0x3F - (addressTag & 0x3F); // take the least significant 6 bits, then "reverse"
  // }
  // return addressTag; // including MCs as Homebases
  // // return getIcntID(addressTag % 56); // not including MCs as Homebases
  ////

  ////
  // addressTag = address & 0x3F0000; // take 6 bits before the 16 least significant bits
  // addressTag = addressTag >> 16; // shift right 16 bits
  // return addressTag; // including MCs as Homebases
  // // return getIcntID(addressTag % 56); // not including MCs as Homebases
  ////
}

unsigned InterconnectInterface::getHomebase(mem_fetch * mf) {
  long long unsigned mfAddress = mf->get_addr();
  return getHomebase(mfAddress);
}



InterconnectInterface* InterconnectInterface::New(const char* const config_file)
{
  if (! config_file ) {
    cout << "Interconnect Requires a configfile" << endl;
    exit (-1);
  }
  InterconnectInterface* icnt_interface = new InterconnectInterface();
  icnt_interface->_icnt_config = new IntersimConfig();
  icnt_interface->_icnt_config->ParseFile(config_file);

  
  //// Khoa
  //// initialization zone
  icnt_interface->wrongSpaceCount = 0;

  icnt_interface->redirectedCount = 0;
  icnt_interface->changedCount = 0;
  icnt_interface->pushCount = 0;
  icnt_interface->popCount = 0;
  icnt_interface->popCount_noData = 0; 

  icnt_interface->readRequestPushCount = 0;
  icnt_interface->writeRequestPushCount = 0;
  icnt_interface->readReplyPushCount = 0;
  icnt_interface->writeReplyPushCount = 0;

  icnt_interface->readRequestPopCount = 0;
  icnt_interface->writeRequestPopCount = 0;
  icnt_interface->readReplyPopCount = 0;
  icnt_interface->writeReplyPopCount = 0;

  
  icnt_interface->totalPacketHopsCount = 0;
  icnt_interface->totalHops_ReadRequest = 0;
  icnt_interface->totalHops_ReadReply = 0;
  icnt_interface->totalHops_Update = 0;
  icnt_interface->totalLUTRead_Homebase = 0;
  icnt_interface->totalLUTWrite_Homebase = 0;
  icnt_interface->totalLUTRead_MC = 0;
  icnt_interface->totalLUTWrite_MC = 0;
  icnt_interface->totalReadRequestWaitTimeAtMC = 0;
  icnt_interface->totalAllRequestWaitTimeAtMC = 0;
  icnt_interface->totalReplyWaitTimeAtMC = 0;
  
  icnt_interface->totalReadRoundTripTime_ts1 = 0; // Khoa, 2023/03
  icnt_interface->totalReadRoundTripTime_init = 0;
  
  icnt_interface->totalReadRequest = 0;
  icnt_interface->totalWriteRequest = 0;
  icnt_interface->totalReadReply_MC = 0;
  icnt_interface->totalReadReply_peer = 0;

  icnt_interface->numOfDistinctContent = 0;
  icnt_interface->numOfDistinctContent_approx = 0;
  icnt_interface->numOfDistinctContent2 = 0;
  icnt_interface->numOfDistinctContent_approx2 = 0;


  icnt_interface->sameAddressAtSameClusterAccessCountTracking = 0;
  icnt_interface->sameAddressAtOtherClusterAccessCountTracking = 0;
  icnt_interface->sameAddressAtBothClustersAccessCountTracking = 0;
  icnt_interface->sameAddressAtEitherClusterAccessCountTracking = 0;
  icnt_interface->sameAddressAtEitherClusterAccessCountTracking_100 = 0;
  icnt_interface->sameAddressAtEitherClusterAccessCountTracking_200 = 0;

  icnt_interface->sameContentAtSameClusterAccessCountTracking = 0;
  icnt_interface->sameContentAtOtherClusterAccessCountTracking = 0;
  icnt_interface->sameContentAtBothClustersAccessCountTracking = 0;
  icnt_interface->sameContentAtEitherClusterAccessCountTracking = 0;
  icnt_interface->sameContentAtEitherClusterAccessCountTracking_100 = 0;
  icnt_interface->sameContentAtEitherClusterAccessCountTracking_200 = 0;

  icnt_interface->approxContentAtSameClusterAccessCountTracking = 0;
  icnt_interface->approxContentAtOtherClusterAccessCountTracking = 0;
  icnt_interface->approxContentAtBothClustersAccessCountTracking = 0;
  icnt_interface->approxContentAtEitherClusterAccessCountTracking = 0;
  icnt_interface->approxContentAtEitherClusterAccessCountTracking_100 = 0;
  icnt_interface->approxContentAtEitherClusterAccessCountTracking_200 = 0;
  

  vector < unsigned > groupOf4;
  vector < vector < unsigned > > groupOf16;

  groupOf4.push_back(0);
  groupOf4.push_back(1);
  groupOf4.push_back(8);
  groupOf4.push_back(9);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();
  
  groupOf4.push_back(2);
  groupOf4.push_back(3);
  groupOf4.push_back(10);
  groupOf4.push_back(11);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();

  groupOf4.push_back(16);
  groupOf4.push_back(17);
  groupOf4.push_back(24);
  groupOf4.push_back(25);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();

  groupOf4.push_back(18);
  groupOf4.push_back(19);
  groupOf4.push_back(26);
  groupOf4.push_back(27);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();
  
  icnt_interface->homebaseMap.push_back(groupOf16);
  groupOf16.clear();

  groupOf4.push_back(4);
  groupOf4.push_back(5);
  groupOf4.push_back(12);
  groupOf4.push_back(13);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();
  
  groupOf4.push_back(6);
  groupOf4.push_back(7);
  groupOf4.push_back(14);
  groupOf4.push_back(15);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();

  groupOf4.push_back(20);
  groupOf4.push_back(21);
  groupOf4.push_back(28);
  groupOf4.push_back(29);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();

  groupOf4.push_back(22);
  groupOf4.push_back(23);
  groupOf4.push_back(30);
  groupOf4.push_back(31);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();
  
  icnt_interface->homebaseMap.push_back(groupOf16);
  groupOf16.clear();

  groupOf4.push_back(32);
  groupOf4.push_back(33);
  groupOf4.push_back(40);
  groupOf4.push_back(41);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();
  
  groupOf4.push_back(34);
  groupOf4.push_back(35);
  groupOf4.push_back(42);
  groupOf4.push_back(43);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();

  groupOf4.push_back(48);
  groupOf4.push_back(49);
  groupOf4.push_back(56);
  groupOf4.push_back(57);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();

  groupOf4.push_back(50);
  groupOf4.push_back(51);
  groupOf4.push_back(58);
  groupOf4.push_back(59);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();
  
  icnt_interface->homebaseMap.push_back(groupOf16);
  groupOf16.clear();

  groupOf4.push_back(36);
  groupOf4.push_back(37);
  groupOf4.push_back(44);
  groupOf4.push_back(45);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();
  
  groupOf4.push_back(38);
  groupOf4.push_back(39);
  groupOf4.push_back(46);
  groupOf4.push_back(47);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();

  groupOf4.push_back(52);
  groupOf4.push_back(53);
  groupOf4.push_back(60);
  groupOf4.push_back(61);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();

  groupOf4.push_back(54);
  groupOf4.push_back(55);
  groupOf4.push_back(62);
  groupOf4.push_back(63);
  groupOf16.push_back(groupOf4);
  groupOf4.clear();
  
  icnt_interface->homebaseMap.push_back(groupOf16);
  groupOf16.clear();
  ////

  return icnt_interface;
}

InterconnectInterface::InterconnectInterface()
{

}

InterconnectInterface::~InterconnectInterface()
{
  for (int i=0; i<_subnets; ++i) {
    ///Power analysis
    if(_icnt_config->GetInt("sim_power") > 0){
      Power_Module pnet(_net[i], *_icnt_config);
      pnet.run();
    }
    delete _net[i];
  }

  delete _traffic_manager;
  _traffic_manager = NULL;
  delete _icnt_config;
}

void InterconnectInterface::CreateInterconnect(unsigned n_shader, unsigned n_mem)
{
  _n_shader = n_shader;
  _n_mem = n_mem;

  InitializeRoutingMap(*_icnt_config);

  gPrintActivity = (_icnt_config->GetInt("print_activity") > 0);
  gTrace = (_icnt_config->GetInt("viewer_trace") > 0);

  string watch_out_file = _icnt_config->GetStr( "watch_out" );
  if(watch_out_file == "") {
    gWatchOut = NULL;
  } else if(watch_out_file == "-") {
    gWatchOut = &cout;
  } else {
    gWatchOut = new ofstream(watch_out_file.c_str());
  }

  _subnets = _icnt_config->GetInt("subnets");
  assert(_subnets);

  /*To include a new network, must register the network here
   *add an else if statement with the name of the network
   */
  _net.resize(_subnets);
  for (int i = 0; i < _subnets; ++i) {
    ostringstream name;
    name << "network_" << i;
    _net[i] = Network::New( *_icnt_config, name.str() );
  }

  assert(_icnt_config->GetStr("sim_type") == "gpgpusim");
  _traffic_manager = static_cast<GPUTrafficManager*>(TrafficManager::New( *_icnt_config, _net, n_shader )) ;


  _traffic_manager->_m_icnt_i = this;// Khoa, 2022/07/
  redirectPacketPerDestCount.resize(_n_mem + _n_shader);
  receivedPacketPerDestCount.resize(_n_mem + _n_shader);
  routedPacketPerDestCount.resize(_n_mem + _n_shader);
  for (int idx = 0; idx < routedPacketPerDestCount.size(); idx++) {
    routedPacketPerDestCount[idx].resize(_n_mem + _n_shader);
  }
  //// 


  _flit_size = _icnt_config->GetInt( "flit_size" );

  // Config for interface buffers
  if (_icnt_config->GetInt("ejection_buffer_size")) {
    _ejection_buffer_capacity = _icnt_config->GetInt( "ejection_buffer_size" ) ;
  } else {
    _ejection_buffer_capacity = _icnt_config->GetInt( "input_buffer_size" ); // Khoa
    // _ejection_buffer_capacity = _icnt_config->GetInt( "vc_buf_size" ); // Khoa
  }

  _boundary_buffer_capacity = _icnt_config->GetInt( "boundary_buffer_size" ) ;
  assert(_boundary_buffer_capacity);
  if (_icnt_config->GetInt("input_buffer_size")) {
    _input_buffer_capacity = _icnt_config->GetInt("input_buffer_size");
  } else {
    _input_buffer_capacity = 9;
  }
  _vcs = _icnt_config->GetInt("num_vcs");

  _CreateBuffer();
  _CreateNodeMap(_n_shader, _n_mem, _traffic_manager->_nodes, _icnt_config->GetInt("use_map"));
}

void InterconnectInterface::Init()
{
  _traffic_manager->Init();
  // TODO: Should we init _round_robin_turn?
  //       _boundary_buffer, _ejection_buffer and _ejected_flit_queue should be cleared
}

void InterconnectInterface::Push(unsigned input_deviceID, unsigned output_deviceID, void *data, unsigned int size)
{
  // it should have free buffer
  // assert(HasBuffer(input_deviceID, size)); // Khoa, moved down

  DPRINTF(INTERCONNECT, "Sent %d bytes from %d to %d", size, input_deviceID, output_deviceID);
  
  int output_icntID = _node_map[output_deviceID];
  int input_icntID = _node_map[input_deviceID];

#if 0
  cout<< "Call interconnect push input: " << input << " output: " << output << endl;
#endif

  //TODO: move to _IssuePacket
  //TODO: create a Inject and wrap _IssuePacket and _GeneratePacket
  unsigned int n_flits = size / _flit_size + ((size % _flit_size)? 1:0);
  // unsigned int n_flit_s = 3; // Khoa, 2022/07/, 2 x 4 bytes + n x byte  


// Khoa, 2022/07/
  // int subnet;
  // if (_subnets == 1) {
  //   subnet = 0;
  // } else {
  //   if (input_deviceID < _n_shader ) {
  //     subnet = 0;
  //   } else {
  //     subnet = 1;
  //   }
  // }

  //TODO: Remove mem_fetch to reduce dependency
  Flit::FlitType packet_type;
  Flit::FlitType packet_type_s; // Khoa, 2022/07/
  mem_fetch* mf = static_cast<mem_fetch*>(data);

  switch (mf->get_type()) {
    case READ_REQUEST:  packet_type = Flit::READ_REQUEST   ;break;
    case WRITE_REQUEST: packet_type = Flit::WRITE_REQUEST  ;break;
    case READ_REPLY:    packet_type = Flit::READ_REPLY     ;break;
    case WRITE_ACK:     packet_type = Flit::WRITE_REPLY    ;break;
    default:
    	{
    		cout<<"Type "<<mf->get_type()<<" is undefined!"<<endl;
    		assert (0 && "Type is undefined");
    	}
  }


if (packet_type == Flit::READ_REPLY) {
  if (!printNFlit) {
    // printNFlit = true;
    // printf("Number of Flits per read reply: %u\n", n_flits);
  }
  if (input_deviceID >= _n_shader) {
    mf->time_ReadReplySent = _traffic_manager->getTime();
    totalReplyWaitTimeAtMC += (mf->time_ReadReplySent) - (mf->time_ReadReplyInMCQueue);
    // totalReadReply_MC++; // Khoa, 2023/03/28, moved to traffic_manager
  } else {
    totalReadReply_peer++;
  }
  readReplyPushCount++;
} else if (packet_type == Flit::READ_REQUEST) {
  totalReadRequest++; //  == readRequestPushCount ?
  readRequestPushCount++;

  mf->time_ReadRequestInit = _traffic_manager->getTime();
  
} else if (packet_type == Flit::WRITE_REQUEST) {
  totalWriteRequest++;
  writeRequestPushCount++;
} else if (packet_type == Flit::WRITE_REPLY) {
  writeReplyPushCount++;
}

//// Khoa, 2022/06
long long unsigned my_addr;
my_addr = mf->get_addr();

//// early data print out test
// if (packet_type == Flit::READ_REPLY) {

//   memory_space *mem = NULL;
//   memory_space_t m_m_space = mf->get_inst().space;
//   // memory_space_t m_m_space = mf->get_inst().whichspace(my_addr);
//   mf->get_w_inst()->decode_space(m_m_space, mem, my_addr);

//   unsigned my_access_size = mf->get_access_size();
//   char * tBuffer = new char[my_access_size+1];
//   if (m_m_space == global_space) {
//     mem->read(my_addr, my_access_size, tBuffer);
//   }
//   tBuffer[my_access_size] = '\0';
//   unsigned long long * packet_data_hex = new unsigned long long[my_access_size/8];
//   memcpy((void*)packet_data_hex,(void*)tBuffer,my_access_size);

//   // FILE *resOutFile_0 = fopen("testPacketData_.csv", "a");
//   // fprintf(resOutFile_0, "%u,%llu,_%u_%u,0x%08llx,%u,0x",
//   //   mf->get_w_inst()->get_uid(),
//   //   mf->get_w_inst()->issue_cycle,
//   //   mf->get_sid(),
//   //   mf->get_tpc(),
//   //   my_addr,
//   //   mf->get_sub_partition_id()
//   //   );
//   // for (int idx = 0; idx < (my_access_size/8); idx++) {
//   //   fprintf(resOutFile_0, "%016llx", 
//   //     packet_data_hex[idx]);
//   // }
//   // fprintf(resOutFile_0, ",\n");
//   // fclose(resOutFile_0); //


//   // FILE *resOutFile_ = fopen("testNoC_.csv", "a");  
//   // fprintf(resOutFile_, "%u,%llu,,,%u__,%u,%u,%u,0x%08llx,%d,0x%08llx,%u,%u,", 
//   //   mf->get_w_inst()->get_uid(),
//   //   mf->get_w_inst()->issue_cycle,
//   //   mf->get_sid(),
//   //   //mf->get_wid(),
//   //   mf->get_w_inst()->warp_id(),
//   //   mf->get_w_inst()->dynamic_warp_id(),
//   //   mf->get_w_inst()->op,
//   //   mf->get_w_inst()->pc,
//   //   m_m_space.get_type(),
//   //   my_addr,
//   //   my_access_size,
//   //   size
//   //   );

//   // // fprintf(resOutFile_, "%u,noc,%llu,%llu,%u__,%u--%u,%u,0x%llx,%d,0x%llx,%u,%s\n", 
//   // //   mf->get_w_inst()->get_uid(),
//   // //   mf->get_w_inst()->m_config->gpgpu_ctx->the_gpgpusim->g_the_gpu->gpu_sim_cycle, 
//   // //   mf->get_w_inst()->m_config->gpgpu_ctx->the_gpgpusim->g_the_gpu->gpu_tot_sim_cycle,
//   // //   mf->get_sid(),
//   // //   mf->get_wid(),
//   // //   mf->get_w_inst()->warp_id(),
//   // //   mf->get_w_inst()->dynamic_warp_id(),
//   // //   mf->get_w_inst()->pc,
//   // //   m_m_space.get_type(),
//   // //   my_addr,
//   // //   size,
//   // //   tBuffer
//   // //   );

//   // fclose(resOutFile_); 
//   // printBits("testNoC_.csv", my_access_size, tBuffer);
//   // resOutFile_ = fopen("testNoC_.csv", "a");  
//   // fprintf(resOutFile_, "\n");
//   // fclose(resOutFile_); //

//   delete []packet_data_hex;
//   delete []tBuffer;
// } ////

  // Khoa, 2022/07/
  int subnet;
  if (_subnets == 1) {
    subnet = 0;
    // _traffic_manager->_GeneratePacket( input_icntID, -1, 0 /*class*/, _traffic_manager->_time, subnet, n_flits, packet_type, data, output_icntID);
  
  } else {
    if (packet_type == Flit::READ_REQUEST || packet_type == Flit::WRITE_REQUEST) {
      subnet = 0;
      // _traffic_manager->_GeneratePacket( input_icntID, -1, 0 /*class*/, _traffic_manager->_time, subnet, n_flits, packet_type, data, output_icntID);

    } else if (packet_type == Flit::READ_REPLY || packet_type == Flit::WRITE_REPLY) {
      subnet = 1;
      // _traffic_manager->_GeneratePacket( input_icntID, -1, 0 /*class*/, _traffic_manager->_time, subnet, n_flits, packet_type, data, output_icntID);
    }
  } ////

  // it should have free buffer
  assert(HasBuffer(input_deviceID, size, subnet)); // Khoa, 

  //TODO: _include_queuing ?
  // if(mf->get_access_type() == L2_WRBK_ACC || mf->get_access_type() == L1_WRBK_ACC) {
  //   _traffic_manager->_GeneratePacket( input_icntID, -1, 0 /*class*/, _traffic_manager->_time, subnet, n_flits, packet_type, data, output_icntID);
  // } else {
  //   mf->set_reply();
  //   _traffic_manager->_GeneratePacket( input_icntID, -1, 0 /*class*/, _traffic_manager->_time, 1, n_flits, packet_type, data, input_icntID);
  // }
  
  _traffic_manager->_GeneratePacket( input_icntID, -1 /*stype, i.e., packet type for nongpgpu*/, 0 /*class*/, _traffic_manager->_time, subnet, n_flits, packet_type, data, output_icntID);



  

long long unsigned curTime = _traffic_manager->getTime();
std::string exactDataKey;
std::string dataKey;
dataKey = _traffic_manager->makeDataKey(mf, exactDataKey);

std::string dataKey_0 = _traffic_manager->makeDataKey_Approx(mf, 2, 0xFC00);
std::string dataKey_1 = _traffic_manager->makeDataKey_Approx_Average(mf, 2, 0);
std::string dataKey_2 = _traffic_manager->makeDataKey_Approx_Average(mf, 2, 1);

if (dataKey != "") {
  ////// characterization section
  // if (packet_type == Flit::READ_REQUEST) {

  //   sameMC_SameContent_DifferentAddresses[output_icntID][exactDataKey].insert(my_addr);
  //   sameMC_ApproxContent_DifferentAddresses[output_icntID][dataKey].insert(my_addr);
    
  //   sameMC_ApproxContent_DifferentAddresses_0[output_icntID][dataKey_0].insert(my_addr);
  //   sameMC_ApproxContent_DifferentAddresses_1[output_icntID][dataKey_1].insert(my_addr);
  //   sameMC_ApproxContent_DifferentAddresses_2[output_icntID][dataKey_2].insert(my_addr);

  //   contentAddressMC[dataKey][my_addr].insert(output_icntID); // old
  //   contentMCAddress[dataKey][output_icntID].insert(my_addr); // old
    
  // // FILE *resOutFile_q = fopen("testQuick_.csv", "a");
  // // fprintf(resOutFile_q, "0x%08llx,%u\n",
  // //   my_addr,
  // //   output_icntID
  // // );
  // // fclose(resOutFile_q); //

  //   contentAddresses[dataKey].insert( std::make_pair(my_addr, output_icntID) ); // old

  // // FILE *resOutFile_q2 = fopen("testQuick_added_.csv", "a");
  // // fprintf(resOutFile_q2, "0x%08llx,%u\n",
  // //   contentAddresses[dataKey].begin()->first,
  // //   contentAddresses[dataKey].begin()->second
  // // );
  // // fclose(resOutFile_q2); //

  //   contentMCs[dataKey].insert(output_icntID); // old
    
    
  //   if (sameAddress_Total.count(my_addr)) {
  //     sameAddress_Total[my_addr]++;
  //   } else {
  //     sameAddress_Total[my_addr] = 1; // set to 0 would mean counting only duplicated accesses, and set to 1 mean counting all accesses
  //   }


  //   if (sameMC_SameContent_Total[output_icntID].count(exactDataKey)) {
  //     sameMC_SameContent_Total[output_icntID][exactDataKey]++;
  //   } else {
  //     sameMC_SameContent_Total[output_icntID][exactDataKey] = 1; // set to 0 would mean counting only duplicated accesses, and set to 1 mean counting all accesses
  //   }

  //   if (sameMC_ApproxContent_Total[output_icntID].count(dataKey)) {
  //     sameMC_ApproxContent_Total[output_icntID][dataKey]++; 
  //   } else {
  //     sameMC_ApproxContent_Total[output_icntID][dataKey] = 1; // set to 0 would mean counting only duplicated accesses, and set to 1 mean counting all accesses
  //   }

  //   // //// same address group
  //   // if (sameAddress_AtMultiClusters.count(my_addr)) {
  //   //   if (sameAddress_AtMultiClusters[my_addr].count(input_icntID)) {
  //   //     //// same address accessed previously by same cluster
  //   //     sameAddressAtSameClusterAccessCountTracking++;
  //   //     if (sameAddress_AtMultiClusters[my_addr].size() > 1) {
  //   //       //// same address also previously accessed by other cluster
  //   //       sameAddressAtBothClustersAccessCountTracking++;
  //   //     }
  //   //   } else {
  //   //     //// same address accessed previously by other clusters
  //   //     if (sameAddress_LastAccessTime.count(my_addr)) {
  //   //       //// same address accessed previously, timeStamp available, *logically redundant as we shouldn't need this, but put in for safe check*
  //   //       if ((sameAddress_LastAccessTime[my_addr] + 300) > curTime) {
  //   //         //// last access within last 300 cycles
  //   //         sameAddressAtOtherClusterAccessCountTracking++;
  //   //       }
  //   //     }
  //   //   }
  //   //   if (sameAddress_LastAccessTime.count(my_addr)) {
  //   //     //// timeStamp available, *again, redundant*
  //   //     if ((sameAddress_LastAccessTime[my_addr] + 300) > curTime) {
  //   //       //// last access within last 300 cycles
  //   //       sameAddressAtEitherClusterAccessCountTracking++;
  //   //     }
  //   //   }
  //   // }
  //   // // sameAddress_LastAccessTime[my_addr] = curTime; 
  //   // sameAddress_AtMultiClusters[my_addr].insert(input_icntID);
    
  //   // //// same content group
  //   // if (sameMC_SameContent_AtMultiClusters[output_icntID].count(exactDataKey)) {
  //   //   if (sameMC_SameContent_AtMultiClusters[output_icntID][exactDataKey].count(input_icntID)) {
  //   //     sameContentAtSameClusterAccessCountTracking++;
  //   //     if (sameMC_SameContent_AtMultiClusters[output_icntID][exactDataKey].size() > 1) {
  //   //       sameContentAtBothClustersAccessCountTracking++;
  //   //     }
  //   //   } else {
  //   //     if (sameMC_SameContent_LastAccessTime[output_icntID].count(exactDataKey)) {
  //   //       if ((sameMC_SameContent_LastAccessTime[output_icntID][exactDataKey] + 300) > curTime) {
  //   //         sameContentAtOtherClusterAccessCountTracking++;
  //   //       }
  //   //     }
  //   //   }
  //   //   if (sameMC_SameContent_LastAccessTime[output_icntID].count(exactDataKey)) {
  //   //     if ((sameMC_SameContent_LastAccessTime[output_icntID][exactDataKey] + 300) > curTime) {
  //   //       sameContentAtEitherClusterAccessCountTracking++;
  //   //     }
  //   //   }
  //   // }
  //   // // sameMC_SameContent_LastAccessTime[output_icntID][exactDataKey] = curTime;
  //   // sameMC_SameContent_AtMultiClusters[output_icntID][exactDataKey].insert(input_icntID);

  //   // //// approx content group
  //   // if (sameMC_ApproxContent_AtMultiClusters[output_icntID].count(dataKey)) {
  //   //   if (sameMC_ApproxContent_AtMultiClusters[output_icntID][dataKey].count(input_icntID)) {
  //   //     approxContentAtSameClusterAccessCountTracking++;
  //   //     if (sameMC_ApproxContent_AtMultiClusters[output_icntID][dataKey].size() > 1) {
  //   //       approxContentAtBothClustersAccessCountTracking++;
  //   //     }
  //   //   } else {
  //   //     if (sameMC_ApproxContent_LastAccessTime[output_icntID].count(dataKey)) {
  //   //       if ((sameMC_ApproxContent_LastAccessTime[output_icntID][dataKey] + 300) > curTime) {
  //   //         approxContentAtOtherClusterAccessCountTracking++;
  //   //       }
  //   //     }
  //   //   }
  //   //   if (sameMC_ApproxContent_LastAccessTime[output_icntID].count(dataKey)) {
  //   //     if ((sameMC_ApproxContent_LastAccessTime[output_icntID][dataKey] + 300) > curTime) {
  //   //       approxContentAtEitherClusterAccessCountTracking++;
  //   //     }
  //   //   }
  //   // }
  //   // // sameMC_ApproxContent_LastAccessTime[output_icntID][dataKey] = curTime;
  //   // sameMC_ApproxContent_AtMultiClusters[output_icntID][dataKey].insert(input_icntID);
  //   // ////


  //   if (!contentOfAddressInitial.count(my_addr)) {
  //     contentOfAddressInitial[my_addr] = exactDataKey;
  //   }
  //   if (!approxContentOfAddressInitial.count(my_addr)) {
  //     approxContentOfAddressInitial[my_addr] = dataKey;
  //   }
  //   if(!mcOfAddress.count(my_addr)) {
  //     mcOfAddress[my_addr] = output_icntID;
  //   }

  //   //// request tracking zone ////
  // } else if (packet_type == Flit::WRITE_REQUEST) {
    

  //   if (input_deviceID < _n_shader) {
  //   //// require checkID when redirect
  //     //// for addresses with changed content over time
  //     if (sameAddress_Total.count(my_addr)) {
  //       //// writing to previously accessed address
        
  //       // unsigned mcIcntID = mcOfAddress[my_addr];

  //       contentOfAddressTracing[my_addr][exactDataKey] = curTime;
  //       approxContentOfAddressTracing[my_addr][dataKey] = curTime;


  //       std::string traceExactDataKey = "";
  //       std::string traceApproxDataKey = "";

  //       sameAddress_Total[my_addr]--;

  //       sameAddress_AtMultiClusters[my_addr].erase(input_icntID);

  //       if (contentOfAddressInitial.count(my_addr)) {
  //         traceExactDataKey = contentOfAddressInitial[my_addr];
  //       }
  //       if (traceExactDataKey != "") {
  //         sameMC_SameContent_AtMultiClusters[output_icntID][traceExactDataKey].erase(input_icntID);
  //       }

  //       if (approxContentOfAddressInitial.count(my_addr)) {
  //         traceApproxDataKey = approxContentOfAddressInitial[my_addr];
  //       }
  //       if (traceApproxDataKey != "") {
  //         sameMC_ApproxContent_AtMultiClusters[output_icntID][traceApproxDataKey].erase(input_icntID);      
  //       }   
  //     }
  //     ////
  //   }

  // } else if (packet_type == Flit::READ_REPLY) {

  //   if (input_deviceID >= _n_shader) {
  //     //// require checkID when redirect, because reply can come from non-MC clusters

  //     //// same address group
  //     if (sameAddress_AtMultiClusters.count(my_addr)) {
  //       if (sameAddress_AtMultiClusters[my_addr].count(output_icntID)) {
  //         //// same address accessed previously by same cluster
  //         sameAddressAtSameClusterAccessCountTracking++;
  //         if (sameAddress_AtMultiClusters[my_addr].size() > 1) {
  //           //// same address also previously accessed by other cluster
  //           sameAddressAtBothClustersAccessCountTracking++;
  //         }
  //       } else {
  //         //// same address accessed previously by other clusters
  //         if (sameAddress_LastAccessTime.count(my_addr)) {
  //           //// same address accessed previously, timeStamp available, *logically redundant as we shouldn't need this, but put in for safe check*
  //           if ((sameAddress_LastAccessTime[my_addr] + 300) > curTime) {
  //             //// last access within last 300 cycles
  //             sameAddressAtOtherClusterAccessCountTracking++;
  //           }
  //         }
  //       }
  //       if (sameAddress_LastAccessTime.count(my_addr)) {
  //         //// timeStamp available, *again, redundant*
  //         if ((sameAddress_LastAccessTime[my_addr] + 100) > curTime) {
  //           sameAddressAtEitherClusterAccessCountTracking_100++;
  //           repeatedAccessAddresses_100.insert(my_addr);
  //         } 
  //         if ((sameAddress_LastAccessTime[my_addr] + 200) > curTime) {
  //           sameAddressAtEitherClusterAccessCountTracking_200++;
  //           repeatedAccessAddresses_200.insert(my_addr);
  //         } 
  //         if ((sameAddress_LastAccessTime[my_addr] + 300) > curTime) {
  //           //// last access within last 300 cycles
  //           sameAddressAtEitherClusterAccessCountTracking++;
  //           repeatedAccessAddresses.insert(my_addr);
  //         }
  //       }
  //     }
  //     sameAddress_LastAccessTime[my_addr] = curTime; 
  //     sameAddress_AtMultiClusters[my_addr].insert(output_icntID);
    
  //     //// same content group
  //     if (sameMC_SameContent_AtMultiClusters[input_icntID].count(exactDataKey)) {
  //       if (sameMC_SameContent_AtMultiClusters[input_icntID][exactDataKey].count(output_icntID)) {
  //         sameContentAtSameClusterAccessCountTracking++;
  //         if (sameMC_SameContent_AtMultiClusters[input_icntID][exactDataKey].size() > 1) {
  //           sameContentAtBothClustersAccessCountTracking++;
  //         }
  //       } else {
  //         if (sameMC_SameContent_LastAccessTime[input_icntID].count(exactDataKey)) {
  //           if ((sameMC_SameContent_LastAccessTime[input_icntID][exactDataKey] + 300) > curTime) {
  //             sameContentAtOtherClusterAccessCountTracking++;
  //           }
  //         }
  //       }
  //       if (sameMC_SameContent_LastAccessTime[input_icntID].count(exactDataKey)) {
  //         if ((sameMC_SameContent_LastAccessTime[input_icntID][exactDataKey] + 100) > curTime) {
  //           sameContentAtEitherClusterAccessCountTracking_100++;
  //           repeatedAccessAddresses_ExactContent_100.insert(my_addr);
  //         } 
  //         if ((sameMC_SameContent_LastAccessTime[input_icntID][exactDataKey] + 200) > curTime) {
  //           sameContentAtEitherClusterAccessCountTracking_200++;
  //           repeatedAccessAddresses_ExactContent_200.insert(my_addr);
  //         } 
  //         if ((sameMC_SameContent_LastAccessTime[input_icntID][exactDataKey] + 300) > curTime) {
  //           sameContentAtEitherClusterAccessCountTracking++;
  //           repeatedAccessAddresses_ExactContent.insert(my_addr);
  //         }
  //       }
  //     }
  //     sameMC_SameContent_LastAccessTime[input_icntID][exactDataKey] = curTime;
  //     sameMC_SameContent_AtMultiClusters[input_icntID][exactDataKey].insert(output_icntID);

  //     //// approx content group
  //     if (sameMC_ApproxContent_AtMultiClusters[input_icntID].count(dataKey)) {
  //       if (sameMC_ApproxContent_AtMultiClusters[input_icntID][dataKey].count(output_icntID)) {
  //         approxContentAtSameClusterAccessCountTracking++;
  //         if (sameMC_ApproxContent_AtMultiClusters[input_icntID][dataKey].size() > 1) {
  //           approxContentAtBothClustersAccessCountTracking++;
  //         }
  //       } else {
  //         if (sameMC_ApproxContent_LastAccessTime[input_icntID].count(dataKey)) {
  //           if ((sameMC_ApproxContent_LastAccessTime[input_icntID][dataKey] + 300) > curTime) {
  //             approxContentAtOtherClusterAccessCountTracking++;
  //           }
  //         }
  //       }
  //       if (sameMC_ApproxContent_LastAccessTime[input_icntID].count(dataKey)) {
  //         if ((sameMC_ApproxContent_LastAccessTime[input_icntID][dataKey] + 100) > curTime) {
  //           approxContentAtEitherClusterAccessCountTracking_100++;
  //           repeatedAccessAddresses_ApproxContent_100.insert(my_addr);
  //         } 
  //         if ((sameMC_ApproxContent_LastAccessTime[input_icntID][dataKey] + 200) > curTime) {
  //           approxContentAtEitherClusterAccessCountTracking_200++;
  //           repeatedAccessAddresses_ApproxContent_200.insert(my_addr);
  //         } 
  //         if ((sameMC_ApproxContent_LastAccessTime[input_icntID][dataKey] + 300) > curTime) {
  //           approxContentAtEitherClusterAccessCountTracking++;
  //           repeatedAccessAddresses_ApproxContent.insert(my_addr);
  //         }
  //       }
  //     }
  //     sameMC_ApproxContent_LastAccessTime[input_icntID][dataKey] = curTime;
  //     sameMC_ApproxContent_AtMultiClusters[input_icntID][dataKey].insert(output_icntID);
      
      
  //     // sameMC_SameContent_LastAccessTime[input_icntID][exactDataKey] = curTime;
  //     // sameMC_ApproxContent_LastAccessTime[input_icntID][dataKey] = curTime;
  //     // sameAddress_LastAccessTime[my_addr] = curTime;
  //   }

  // } else if (packet_type == Flit::WRITE_REPLY) {
  // }
  ////// end  characterization
} else {
  wrongSpaceCount++;
}

pushCount++;
//// end tracking zone


#if DOUB
  cout <<"Traffic[" << subnet << "] (mapped) sending form "<< input_icntID << " to " << output_icntID << endl;
#endif
//  }
}

void* InterconnectInterface::Pop(unsigned deviceID, unsigned subnet, long long unsigned curCycle)
// void* InterconnectInterface::Pop(unsigned deviceID)
{
  int icntID = _node_map[deviceID];
  // if (deviceID == 0) {
  //       FILE *resOutFile_tt = fopen("testTimeMatch_.csv", "a");
  //       fprintf(resOutFile_tt, 
  //         "node # %03u,%llu,%llu\n", 
  //           icntID,
  //           curCycle,
  //           _traffic_manager->getTime()
  //       );
  //       fclose(resOutFile_tt);
  // }

#if DEBUG
  cout<< "Call interconnect POP  " << output <<endl;
#endif

  void* data = NULL;

  // 0-_n_shader-1 indicates reply(network 1), otherwise request(network 0)
  // int subnet = 0;
  // if (deviceID < _n_shader)
  //   subnet = 1;

  int turn = _round_robin_turn[subnet][icntID];
  for (int vc=0;(vc<_vcs) && (data==NULL);vc++) {
    if (_boundary_buffer[subnet][icntID][turn].HasPacket()) {
      data = _boundary_buffer[subnet][icntID][turn].PopPacket();
    }
    turn++;
    if (turn == _vcs) turn = 0;
  }
  if (data) {
    _round_robin_turn[subnet][icntID] = turn;



    // Khoa, 2022/07/
    mem_fetch* mft = static_cast<mem_fetch* >(data);
    unsigned long long blAddress = mft->get_addr();
    // std::string exactDataKey;
    // std::string dataKey = _traffic_manager->makeDataKey(mft, exactDataKey);

    // if (mft->get_request_uid() == 49017) {
    //   FILE * resOutFile_tracing = fopen("testPacketPop_.csv", "a");
    //   fprintf(resOutFile_tracing, 
    //     "%u,%u (%u)",
    //       mft->get_request_uid(),
    //       icntID,
    //       deviceID
    //   );
    //   fclose(resOutFile_tracing);
    // }

    if (subnet == 0) {
      if (deviceID >= _n_shader) {
        //// read request reach MC
        mft->time_RequestInMCQueue = _traffic_manager->getTime();
        if (mft->time_RequestReceived > 0) {
          totalAllRequestWaitTimeAtMC += ((mft->time_RequestInMCQueue) - (mft->time_RequestReceived));
          if ((mft->get_type() == READ_REQUEST)) {
            totalReadRequestWaitTimeAtMC += ((mft->time_RequestInMCQueue) - (mft->time_RequestReceived));
          }
        }
        
      }
    }

    // received packet counts
    receivedPacketPerDestCount[icntID]++;
    // if (deviceID >= _n_shader) {
    //   // testing
    //   redirectPacketPerDestCount[icntID]++;
    // }
    

    if (mft->get_type() == READ_REQUEST) {
      readRequestPopCount++;
    } else if (mft->get_type() == WRITE_REQUEST) {
      writeRequestPopCount++;
    } else if (mft->get_type() == READ_REPLY) {
      readReplyPopCount++;
      // Khoa, 2023/03
      totalReadRoundTripTime_ts1 += (_traffic_manager->getTime() - mft->get_timestamp());
      totalReadRoundTripTime_init += (_traffic_manager->getTime() - mft->time_ReadRequestInit);
    } else if (mft->get_type() == WRITE_ACK) {
      writeReplyPopCount++;
    }


  } else { // Khoa
    popCount_noData++;
  }

  
  popCount++;


  return data;

}


// Khoa
void InterconnectInterface::printLUT() {


for (int idx1 = 0; idx1 < _subnets; idx1++) {
  std::string filePath = "testLinkUtilization_subnet_" + std::to_string(idx1) + "_.csv";
  // FILE *resOutFile_linkUtil = fopen("testLinkUtilization_.csv", "a");
  FILE *resOutFile_linkUtil = fopen(filePath.c_str(), "a");
  fprintf(resOutFile_linkUtil,
    "Num Of Nodes:,%d,,Num of Routers:,%d,,Num of Classes:,%d,Subnet:,%d\n",
      _net[0]->NumNodes(),
      _net[0]->NumRouters(),
      _net[0]->GetRouter(0)->_classes,
      idx1
  );
  
  std::string filePath2 = "testIcntPower_subnet_" + std::to_string(idx1) + "_.csv";
  FILE *resOutFile_IcntPower = fopen(filePath2.c_str(), "a");
  fprintf(resOutFile_IcntPower,
    "Subnet:,%d,Num Of Nodes:,%d,,Num of Routers:,%d\n",
      idx1,
      _net[0]->NumNodes(),
      _net[0]->NumRouters()
  );
  
  for (int idx2 = 0; idx2 < _net[idx1]->NumRouters(); idx2++) {
    Router * tRouter = _net[idx1]->GetRouter(idx2);
    
    for (int idx3i = 0; idx3i < tRouter->_inputs; idx3i++) {
      for (int idx4i = 0; idx4i < tRouter->_classes; idx4i++) {
        fprintf(resOutFile_linkUtil,
          "%03d,in,%d,%d,_,%d,%d\n",
            tRouter->_id,
            idx3i,
            idx4i,
            tRouter->_input_channels[idx3i]->getActive(idx4i),
            tRouter->_input_channels[idx3i]->getIdle()
        );
      }  
    }
    
    for (int idx3o = 0; idx3o < tRouter->_outputs; idx3o++) {
      for (int idx4o = 0; idx4o < tRouter->_classes; idx4o++) {
        fprintf(resOutFile_linkUtil,
          "%03d,out,%d,%d,_,%d,%d\n",
            tRouter->_id,
            idx3o,
            idx4o,
            tRouter->_output_channels[idx3o]->getActive(idx4o),
            tRouter->_output_channels[idx3o]->getIdle()
        );
      }  
    }
    fflush(resOutFile_linkUtil);
    
    fprintf(resOutFile_IcntPower,
      "Router:,%03d,xbar:,%llu,bufRd:,%llu,bufWr:,%llu\n",
        tRouter->_id,
        tRouter->_crossbarUseCounter,
        tRouter->_vcBufferReadCounter,
        tRouter->_vcBufferWriteCounter
    );
    fflush(resOutFile_IcntPower);
  } // end for num of routers/nodes

  // for (int idx2 = 0; idx2 < _net[idx1]->NumChannels(); idx2++) {
  //   fprintf(resOutFile_linkUtil,
  //     "%d,%d,%d\n",
  //       idx2,
  //       _net[idx1]->GetChannels()[idx2]->getActive(0),
  //       _net[idx1]->GetChannels()[idx2]->getIdle()
  //   );
  // }
  fprintf(resOutFile_linkUtil,
    "----------------------------------------------------------------\n"
  );
  fclose(resOutFile_linkUtil);
  
  fprintf(resOutFile_IcntPower,
    "----------------------------------------------------------------\n"
  );
  fclose(resOutFile_IcntPower);
} // end for subnets

FILE *resOutFile_trt = fopen("testReadRoundTripTime_.csv", "a");
fprintf(resOutFile_trt,
  "%llu,%llu,,%u,%u",
    totalReadRoundTripTime_ts1,
    totalReadRoundTripTime_init,
    readRequestPushCount, 
    readReplyPushCount
);
fclose(resOutFile_trt);

FILE *resOutFile_haw = fopen("testHopsAndWaittime_.csv", "a");
// printf("TOTAL PACKET HOPS COUNT:%u\n", totalPacketHopsCount);
fprintf(resOutFile_haw,
  "%u,%u,,%u,%u,,%u,%u,%u,_",
    totalReadRequestWaitTimeAtMC, totalReadRequest,
    totalReplyWaitTimeAtMC, totalReadReply_MC, totalReadReply_peer,
    totalWriteRequest, 
    totalAllRequestWaitTimeAtMC
);
fprintf(resOutFile_haw,
  ",%u,%u,%u,%u,_,%u,%u,%u,%u,wrong_space_count:%u\n",
    totalPacketHopsCount, totalHops_ReadRequest, totalHops_ReadReply, totalHops_Update,
    totalLUTRead_Homebase,
    totalLUTWrite_Homebase,
    totalLUTRead_MC,
    totalLUTWrite_MC,
    wrongSpaceCount
);

fclose(resOutFile_haw);



  printf("printLUT() break 1\n");
  fflush(stdout);

  FILE *resOutFile_cc = fopen("testChangedPacketCount_.csv", "a");
  // fprintf(resOutFile_cc, 
  //   "%u,%u,%u,%u\n", 
  //     redirectedCount,
  //     changedCount,
  //     readRequestPushCount,
  //     pushCount
  // );
  fprintf(resOutFile_cc, 
    "_,%lld,redir:%u,%u,_,push:%u,pop:%u,pop_noData:%u,", 
      _traffic_manager->getTime(),
      redirectedCount,
      redirectedPackets.size(),
      pushCount,
      popCount,
      popCount_noData
  );
  fprintf(resOutFile_cc, 
    "_,rReq:%u,wReq:%u,rRes:%u,wRes:%u,rReq:%u,wReq:%u,rRes:%u,wRes:%u,changed:%u\n", 
      readRequestPushCount,
      writeRequestPushCount,
      readReplyPushCount,
      writeReplyPushCount,
      readRequestPopCount,
      writeRequestPopCount,
      readReplyPopCount,
      writeReplyPopCount,
      changedCount
  );
  fclose(resOutFile_cc);
  //// note*remind* exact dataKey vs. approx dataKey
  
  // unordered_map< std::string, std::set< unsigned > >::iterator iter_cMcs;
  // std::set<unsigned>::iterator iter_cMcs_Set;
  // std::unordered_map< std::string, std::set< pair <long long unsigned, unsigned> > >::iterator iter_cAs;
  // std::set< pair <long long unsigned, unsigned> >::iterator iter_cAs_Set;
  // FILE *resOutFile_cMcs = fopen("testContentMCs_.csv", "a");
  // for ( iter_cMcs = contentMCs.begin();
  //       iter_cMcs != contentMCs.end();
  //       iter_cMcs++ ) {
  //   // FILE *resOutFile_cMcs = fopen("testContentMCs_.csv", "a");
    
  //   fprintf(resOutFile_cMcs, "%s,%u,", 
  //       iter_cMcs->first.c_str(),
  //       iter_cMcs->second.size() 
  //   );
  //   for ( iter_cMcs_Set = iter_cMcs->second.begin();
  //         iter_cMcs_Set != iter_cMcs->second.end();
  //         iter_cMcs_Set++ ) {
  //     fprintf(resOutFile_cMcs, ",%03u", 
  //       (*iter_cMcs_Set) 
  //     );
  //   }
  //   fprintf(resOutFile_cMcs, "\n");

  //   // fclose(resOutFile_cMcs);  
  // }
  // fclose(resOutFile_cMcs);


  // FILE *resOutFile_cAs = fopen("testContentAddressesMCs_.csv", "a");
  // for ( iter_cAs = contentAddresses.begin(); 
  //       iter_cAs != contentAddresses.end();
  //       iter_cAs++ ) {
  //   fprintf(resOutFile_cAs, "%s,%u,",
  //     iter_cAs->first.c_str(),
  //     iter_cAs->second.size()
  //   );
  //   for ( iter_cAs_Set = iter_cAs->second.begin(); 
  //         iter_cAs_Set != iter_cAs->second.end();
  //         iter_cAs_Set++ ) {
  //     fprintf(resOutFile_cAs, ",0x%08llx_%u",
  //       iter_cAs_Set->first,
  //       iter_cAs_Set->second
  //     );
  //   }
  //   fprintf(resOutFile_cAs, "\n");
  // }
  // fclose(resOutFile_cAs);


  // unordered_map< std::string, std::unordered_map< long long unsigned, std::set< unsigned > > >::iterator iter_cAMcs;
  // std::unordered_map< long long unsigned, std::set< unsigned > >::iterator iter_cAMcs_A; // though each address should belong to only 1 MC
  // std::set<unsigned>::iterator iter_cAMcs_Set;
  // FILE *resOutFile_cAMcs = fopen("testContentAddressesMCs_.csv", "a");
  //
  //  
  // fclose(resOutFile_cAMcs);


  printf("printLUT() break 2\n");
  fflush(stdout);

//// latest
  FILE *resOutFile_d = fopen("testRedirectPacketPerDest_.csv", "a");
  for (unsigned idx = 0; idx < (_n_shader + _n_mem); idx++ ) {
    if ( getDeviceID(idx) < _n_shader ) {
      fprintf(resOutFile_d, "cluster node# %03u/%u:%u,%u,%u\n", 
        idx,
        _traffic_manager->NoCLUT_memPart.size(), 
        (_n_shader + _n_mem),
        redirectPacketPerDestCount[idx],
        receivedPacketPerDestCount[idx]
      );
    } else {
      fprintf(resOutFile_d, "mem node# %03u/%u:%u,%u,%u\n", 
        idx, 
        _traffic_manager->NoCLUT_memPart.size(), 
        (_n_shader + _n_mem),
        redirectPacketPerDestCount[idx],
        receivedPacketPerDestCount[idx]
      );
    }

  }
  fprintf(resOutFile_d, "\n\n");
  fclose(resOutFile_d);

  printf("printLUT() break 3\n");
  fflush(stdout);

  FILE *resOutFile_lutSize = fopen("testLUTSize_n_.csv", "a");
  for (unsigned idx = 0; idx < (_n_shader + _n_mem); idx++ ) {
    if ( getDeviceID(idx) < _n_shader ) {
      fprintf(resOutFile_lutSize, "cluster node,ID# %03u/%u:%u,memPart:%u,cluster:%u\n", 
        idx,
        _traffic_manager->NoCLUT_memPart.size(), 
        (_n_shader + _n_mem),
        _traffic_manager->NoCLUT_memPart[idx].size(),
        _traffic_manager->NoCLUT_cluster[idx].size()
      );
    } else {
      fprintf(resOutFile_lutSize, "mem node,ID# %03u/%u:%u,memPart:%u,cluster:%u\n", 
        idx, 
        _traffic_manager->NoCLUT_memPart.size(), 
        (_n_shader + _n_mem),
        _traffic_manager->NoCLUT_memPart[idx].size(),
        _traffic_manager->NoCLUT_cluster[idx].size()
      );
    }

  }
  fprintf(resOutFile_lutSize, "\n\n");
  fclose(resOutFile_lutSize);
////

  // unsigned nodeRecordCount;
  // unordered_map< unsigned long long, unsigned >::iterator iter_cluster_s;
  // unordered_map< unsigned long long, LUTRecord >::iterator iter_cluster;
  // unordered_map< std::string, std::set< unsigned > >::iterator iter_mP;
  // unordered_map< std::string, unsigned  >::iterator iter_mP_s;

  // FILE *resOutFile_c = fopen("testNoCLUT_cluster_.csv", "a");
  // for (unsigned idx = 0; idx < _traffic_manager->NoCLUT_cluster_s.size(); idx++ ) {
  //   nodeRecordCount = 0;
  //   for (iter_cluster_s = ((_traffic_manager->NoCLUT_cluster_s)[idx]).begin(); 
  //         iter_cluster_s != ((_traffic_manager->NoCLUT_cluster_s)[idx]).end(); 
  //         iter_cluster_s++ ) {
  //     nodeRecordCount += 1;
  //   }

  //   if ( getDeviceID(idx) < _n_shader ) {
  //     fprintf(resOutFile_c, "cluster node# %03u/%u:%u,%u,%u\n", 
  //       idx,
  //       _traffic_manager->NoCLUT_cluster_s.size(), 
  //       (_n_shader + _n_mem),
  //       _traffic_manager->NoCLUT_cluster_s[idx].size(),
  //       nodeRecordCount
  //       );
  //   } else {
  //     fprintf(resOutFile_c, "mem node# %03u/%u:%u,%u,%u\n", 
  //       idx, 
  //       _traffic_manager->NoCLUT_cluster_s.size(), 
  //       (_n_shader + _n_mem),
  //       _traffic_manager->NoCLUT_cluster_s[idx].size(),
  //       nodeRecordCount
  //     );
  //   }

  //   cout << "TEST::NODE# " << idx << "," 
  //     /*<< _traffic_manager->NoCLUT_cluster_s[idx].begin()->second << "," */
  //     << ((_traffic_manager->NoCLUT_cluster_s)[idx]).size() << ","
  //     << ((_traffic_manager->NoCLUT_cluster_s)[idx]).max_size() << ","
  //     << nodeRecordCount << "\n";

  // }
  // fclose(resOutFile_c);


  // FILE *resOutFile_c = fopen("testNoCLUT_cluster_.csv", "a");
  // for (unsigned idx = 0; idx < (_n_shader + _n_mem); idx++ ) {
  //   nodeRecordCount = 0;
  //   for (iter_cluster = _traffic_manager->NoCLUT_cluster[idx].begin(); 
  //         iter_cluster != _traffic_manager->NoCLUT_cluster[idx].end(); 
  //         iter_cluster++ ) {
  //   nodeRecordCount += (iter_cluster->second).size();
  //   }

  //   if ( getDeviceID(idx) < _n_shader ) {
  //     fprintf(resOutFile_c, "cluster node# %03u/%u:%u,%u,%u\n", 
  //       idx,
  //       _traffic_manager->NoCLUT_cluster.size(), 
  //       (_n_shader + _n_mem),
  //       _traffic_manager->NoCLUT_cluster[idx].size(),
  //       nodeRecordCount
  //       );
  //   } else {
  //     fprintf(resOutFile_c, "mem node# %03u/%u:%u,%u,%u\n", 
  //       idx, 
  //       _traffic_manager->NoCLUT_cluster.size(), 
  //       (_n_shader + _n_mem),
  //       _traffic_manager->NoCLUT_cluster[idx].size(),
  //       nodeRecordCount
  //     );
  //   }

  //   // cout << "TEST::NODE# " << idx << "," <<  _traffic_manager->NoCLUT_cluster_s[idx].begin()->second << "," << nodeRecordCount << "\n";

  // }
  // fclose(resOutFile_c);




  // FILE *resOutFile_m = fopen("testNoCLUT_memPart_.csv", "a");
  // for (unsigned idx = 0; idx < (_n_shader + _n_mem); idx++ ) {
  //   nodeRecordCount = 0;
  //   for (iter_mP_s = _traffic_manager->NoCLUT_memPart_s[idx].begin(); 
  //         iter_mP_s != _traffic_manager->NoCLUT_memPart_s[idx].end(); 
  //         iter_mP_s++ ) {
  //           nodeRecordCount += 1;
  //   }

  //   if ( getDeviceID(idx) < _n_shader ) {
  //     fprintf(resOutFile_m, "cluster node# %03u/%u:%u,%u,%u\n", 
  //       idx,
  //       _traffic_manager->NoCLUT_memPart_s.size(), 
  //       (_n_shader + _n_mem),
  //       _traffic_manager->NoCLUT_memPart_s[idx].size(),
  //       nodeRecordCount
  //     );
  //   } else {
  //     fprintf(resOutFile_m, "mem node# %03u/%u:%u,%u,%u\n", 
  //       idx, 
  //       _traffic_manager->NoCLUT_memPart_s.size(), 
  //       (_n_shader + _n_mem),
  //       _traffic_manager->NoCLUT_memPart_s[idx].size(),
  //       nodeRecordCount
  //     );
  //   }
  // }
  // fclose(resOutFile_m);


  // FILE *resOutFile_m = fopen("testNoCLUT_memPart_.csv", "a");
  // for (unsigned idx = 0; idx < (_n_shader + _n_mem); idx++ ) {
  //   nodeRecordCount = 0;
  //   for (iter_mP = _traffic_manager->NoCLUT_memPart[idx].begin(); 
  //         iter_mP != _traffic_manager->NoCLUT_memPart[idx].end(); 
  //         iter_mP++ ) {
  //           nodeRecordCount += (iter_mP->second).size();
  //   }

  //   if ( getDeviceID(idx) < _n_shader ) {
  //     fprintf(resOutFile_m, "cluster node# %03u/%u:%u,%u,%u\n", 
  //       idx,
  //       _traffic_manager->NoCLUT_memPart.size(), 
  //       (_n_shader + _n_mem),
  //       _traffic_manager->NoCLUT_memPart[idx].size(),
  //       nodeRecordCount
  //     );
  //   } else {
  //     fprintf(resOutFile_m, "mem node# %03u/%u:%u,%u,%u\n", 
  //       idx, 
  //       _traffic_manager->NoCLUT_memPart.size(), 
  //       (_n_shader + _n_mem),
  //       _traffic_manager->NoCLUT_memPart[idx].size(),
  //       nodeRecordCount
  //     );
  //   }
  //   // cout << "TEST::NODE# " << idx << "," <<  _traffic_manager->NoCLUT_memPart[idx].size() << "," << nodeRecordCount << "\n";
  // }
  // fclose(resOutFile_m);




  // FILE *resOutFile = fopen("test__cluster_.csv", "w");
   
  // printLUT1(resOutFile);
  
  // fclose(resOutFile);
  
  // FILE *resOutFile2 = fopen("test__memP_.csv", "w");
  
  // printLUT2(resOutFile2);

  // fclose(resOutFile2);


  // printLUT1();
  // printLUT2();
}

// void InterconnectInterface::printLUT1(FILE *resOutFile_c) {
void InterconnectInterface::printLUT1() {
  
  //// characterization
  unsigned totalNumOfAddresses = sameAddress_Total.size();

  unsigned sameAddressRequest_numOfAddresses = 0;
  unsigned sameAddressRequest_numOfAccesses = 0;
  unsigned sameAddressRequest_numOfAddresses2 = 0;
  unsigned sameAddressRequest_numOfAccesses2 = 0;
  unsigned sameContentRequest_numOfContent = 0;
  unsigned sameContentRequest_numOfAccesses = 0;
  unsigned approxContentRequest_numOfContent = 0;
  unsigned approxContentRequest_numOfAccesses = 0;

  std::unordered_map< long long unsigned, unsigned >::iterator itrt_Address;
  // std::unordered_map < unsigned, unordered_map < std::string, unsigned > >::iterator itrt_MC;
  // std::unordered_map < std::string, unsigned >::iterator itrt_MCContent;

  for ( itrt_Address = sameAddress_Total.begin();
        itrt_Address != sameAddress_Total.end();
        itrt_Address++ ) {
    if ( (itrt_Address->second > 1) && (!contentOfAddressTracing.count(itrt_Address->first)) ) {
      sameAddressRequest_numOfAccesses += itrt_Address->second; // this counts total number of accesses where the addresses are accessed at least 2 times
      sameAddressRequest_numOfAddresses++; // this counts number of addresses that are accessed at least twice
    }
  }

  printf("printLUT_1() break 1\n");
  fflush(stdout);

  // for ( itrt_MC = sameMC_SameContent_Total.begin();
  //       itrt_MC != sameMC_SameContent_Total.end();
  //       itrt_MC++ ) {

  //   for ( itrt_MCContent = itrt_MC->second.begin();
  //         itrt_MCContent != itrt_MC->second.end();
  //         itrt_MCContent++) {

  //     if (itrt_MCContent->second > 1) {
  //       sameContentRequest += itrt_MCContent->second;
  //     }
  //   }
  // }

  // for ( itrt_MC = sameMC_ApproxContent_Total.begin();
  //       itrt_MC != sameMC_ApproxContent_Total.end();
  //       itrt_MC++ ) {

  //   for ( itrt_MCContent = itrt_MC->second.begin();
  //         itrt_MCContent != itrt_MC->second.end();
  //         itrt_MCContent++) {
            
  //     if (itrt_MCContent->second > 1) {
  //       approxContentRequest += itrt_MCContent->second;
  //     }
  //   }
  // }


  std::unordered_map< long long unsigned, std::set< unsigned > >::iterator itrt2_Address;
  std::unordered_map < unsigned, unordered_map < std::string, set< unsigned > > >::iterator itrt2_MC;
  std::unordered_map < std::string, set< unsigned> >::iterator itrt2_MCContent;
  
  for ( itrt2_Address = sameAddress_AtMultiClusters.begin();
        itrt2_Address != sameAddress_AtMultiClusters.end();
        itrt2_Address++ ) {
    // printf("printLUT_1() break 1a, %08llx\n", itrt2_Address->first);
    // fflush(stdout);

    if ( (itrt2_Address->second.size() > 1) && (!contentOfAddressTracing.count(itrt2_Address->first)) ) {
      // printf("printLUT_1() break 1b, %08llx, %u\n", itrt2_Address->first, sameAddress_Total[itrt2_Address->first]);
      // fflush(stdout);
      
      sameAddressRequest_numOfAccesses2 += sameAddress_Total[itrt2_Address->first];
      // sameAddressRequest_numOfAccesses2 += itrt2_Address->second.size();
      sameAddressRequest_numOfAddresses2++;
    }
  }

  printf("printLUT_1() break 2\n");
  fflush(stdout);
  unsigned breakCounter = 0;
  unsigned breakCounter2 = 0;
  for ( itrt2_MC = sameMC_SameContent_AtMultiClusters.begin();
        itrt2_MC != sameMC_SameContent_AtMultiClusters.end();
        itrt2_MC++ ) {

    // printf("printLUT_1() break 2a, MC# %u, num of distinct content: %u\n", itrt2_MC->first, itrt2_MC->second.size());
    // fflush(stdout);

    numOfDistinctContent += itrt2_MC->second.size();
    
    // printf("printLUT_1() break 2b, MC# %u, num of distinct content 2: %u\n", itrt2_MC->first, sameMC_SameContent_Total[itrt2_MC->first].size());
    // fflush(stdout);

    numOfDistinctContent2 += sameMC_SameContent_Total[itrt2_MC->first].size();
    
    // printf("printLUT_1() break 2c, MC# %u\n", itrt2_MC->first);
    // fflush(stdout);
    
    for ( itrt2_MCContent = itrt2_MC->second.begin();
          itrt2_MCContent != itrt2_MC->second.end();
          itrt2_MCContent++ ) {
            
      // printf("printLUT_1() break 2d, MC# %u, %s\n", itrt2_MC->first, itrt2_MCContent->first.c_str());
      // fflush(stdout);
      
      if (itrt2_MCContent->second.size() > 1) {

        // printf("printLUT_1() break 2e, MC# %u, %s, %u\n", itrt2_MC->first, itrt2_MCContent->first.c_str(), sameMC_SameContent_Total[itrt2_MC->first][itrt2_MCContent->first]);
        // fflush(stdout);
      
        sameContentRequest_numOfAccesses += sameMC_SameContent_Total[itrt2_MC->first][itrt2_MCContent->first];
        // sameContentRequest_numOfAccesses += itrt2_MCContent->second.size();

        // printf("printLUT_1() break 2f, MC# %u, %s\n", itrt2_MC->first, itrt2_MCContent->first.c_str());
        // fflush(stdout);

        sameContentRequest_numOfContent++;
      }
    }
  }

  printf("printLUT_1() break 3\n");
  fflush(stdout);

  for ( itrt2_MC = sameMC_ApproxContent_AtMultiClusters.begin();
        itrt2_MC != sameMC_ApproxContent_AtMultiClusters.end();
        itrt2_MC++ ) {
    numOfDistinctContent_approx += itrt2_MC->second.size();
    numOfDistinctContent_approx2 += sameMC_ApproxContent_Total[itrt2_MC->first].size();
    for ( itrt2_MCContent = itrt2_MC->second.begin();
          itrt2_MCContent != itrt2_MC->second.end();
          itrt2_MCContent++ ) {
      
      if (itrt2_MCContent->second.size() > 1) {
        approxContentRequest_numOfAccesses += sameMC_ApproxContent_Total[itrt2_MC->first][itrt2_MCContent->first];
        // approxContentRequest_numOfAccesses += itrt2_MCContent->second.size();
        approxContentRequest_numOfContent++;
      }
    }
  }

  printf("printLUT_1() break 4\n");
  fflush(stdout);

  unsigned addressWithChangedExactContentCount = 0;
  unsigned addressWithChangedApproxContentCount = 0;
  std::unordered_map < long long unsigned, unordered_map <std::string, long long unsigned > >::iterator itrt_Tracing;
  for ( itrt_Tracing = contentOfAddressTracing.begin();
        itrt_Tracing != contentOfAddressTracing.end();
        itrt_Tracing++ ) {
    if (sameAddress_Total.count(itrt_Tracing->first)) {
      addressWithChangedExactContentCount++;
    }
  }
  for ( itrt_Tracing = approxContentOfAddressTracing.begin();
        itrt_Tracing != approxContentOfAddressTracing.end();
        itrt_Tracing++ ) {
    if (sameAddress_Total.count(itrt_Tracing->first)) {
      addressWithChangedApproxContentCount++;
    }
  }

  printf("printLUT_1() break 5\n");
  fflush(stdout);

  unsigned numOfAddressesWithAlias = 0;
  unsigned numOfAddressesWithoutAlias = 0;
  unsigned numOfAddressesWithAlias_approx = 0;
  unsigned numOfAddressesWithoutAlias_approx = 0;

  unsigned numOfAddressesWithAlias_approx_0 = 0;
  unsigned numOfAddressesWithoutAlias_approx_0 = 0;
  unsigned numOfAddressesWithAlias_approx_1 = 0;
  unsigned numOfAddressesWithoutAlias_approx_1 = 0;
  unsigned numOfAddressesWithAlias_approx_2 = 0;
  unsigned numOfAddressesWithoutAlias_approx_2 = 0;

  std::unordered_map < unsigned, unordered_map < std::string, set < long long unsigned > > >::iterator itrt_MCContentAddresses;
  unordered_map < std::string, set < long long unsigned > >::iterator itrt_ContentAddresses;
  for ( itrt_MCContentAddresses = sameMC_SameContent_DifferentAddresses.begin();
        itrt_MCContentAddresses != sameMC_SameContent_DifferentAddresses.end();
        itrt_MCContentAddresses++ ) {
    for ( itrt_ContentAddresses = itrt_MCContentAddresses->second.begin();
          itrt_ContentAddresses != itrt_MCContentAddresses->second.end();
          itrt_ContentAddresses++ ) {
      if (itrt_ContentAddresses->second.size() > 1) {
        numOfAddressesWithAlias += itrt_ContentAddresses->second.size();
      } else {
        numOfAddressesWithoutAlias++;
      }
    }
  }
  for ( itrt_MCContentAddresses = sameMC_ApproxContent_DifferentAddresses.begin();
        itrt_MCContentAddresses != sameMC_ApproxContent_DifferentAddresses.end();
        itrt_MCContentAddresses++ ) {
    for ( itrt_ContentAddresses = itrt_MCContentAddresses->second.begin();
          itrt_ContentAddresses != itrt_MCContentAddresses->second.end();
          itrt_ContentAddresses++ ) {
      if (itrt_ContentAddresses->second.size() > 1) {
        numOfAddressesWithAlias_approx += itrt_ContentAddresses->second.size();
      } else {
        numOfAddressesWithoutAlias_approx++;
      }
    }
  }

  for ( itrt_MCContentAddresses = sameMC_ApproxContent_DifferentAddresses_0.begin();
        itrt_MCContentAddresses != sameMC_ApproxContent_DifferentAddresses_0.end();
        itrt_MCContentAddresses++ ) {
    for ( itrt_ContentAddresses = itrt_MCContentAddresses->second.begin();
          itrt_ContentAddresses != itrt_MCContentAddresses->second.end();
          itrt_ContentAddresses++ ) {
      if (itrt_ContentAddresses->second.size() > 1) {
        numOfAddressesWithAlias_approx_0 += itrt_ContentAddresses->second.size();
      } else {
        numOfAddressesWithoutAlias_approx_0++;
      }
    }
  }

  for ( itrt_MCContentAddresses = sameMC_ApproxContent_DifferentAddresses_1.begin();
        itrt_MCContentAddresses != sameMC_ApproxContent_DifferentAddresses_1.end();
        itrt_MCContentAddresses++ ) {
    for ( itrt_ContentAddresses = itrt_MCContentAddresses->second.begin();
          itrt_ContentAddresses != itrt_MCContentAddresses->second.end();
          itrt_ContentAddresses++ ) {
      if (itrt_ContentAddresses->second.size() > 1) {
        numOfAddressesWithAlias_approx_1 += itrt_ContentAddresses->second.size();
      } else {
        numOfAddressesWithoutAlias_approx_1++;
      }
    }
  }

  for ( itrt_MCContentAddresses = sameMC_ApproxContent_DifferentAddresses_2.begin();
        itrt_MCContentAddresses != sameMC_ApproxContent_DifferentAddresses_2.end();
        itrt_MCContentAddresses++ ) {
    for ( itrt_ContentAddresses = itrt_MCContentAddresses->second.begin();
          itrt_ContentAddresses != itrt_MCContentAddresses->second.end();
          itrt_ContentAddresses++ ) {
      if (itrt_ContentAddresses->second.size() > 1) {
        numOfAddressesWithAlias_approx_2 += itrt_ContentAddresses->second.size();
      } else {
        numOfAddressesWithoutAlias_approx_2++;
      }
    }
  }
  

  FILE *resOutFile_ch = fopen("testCharacterization_.csv", "a");
  // fprintf(resOutFile_ch, 
  //   ",Time,Total Read Requests:Total Number of Requested Addresses,Same Address Requests,Same Address at Multi Clusters Request,Same Content Requests,Approx Content Requests,,Same Address At Same Cluster,Same Address At Other Cluster,Same Content At Same Cluster,Same Content At Other Cluster,Approx Content At Same Cluster,Approx Content At Other Cluster\n");
  fprintf(resOutFile_ch, 
    ",%lld,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", 
      _traffic_manager->getTime(),
      totalReadRequest, totalNumOfAddresses,
      sameAddressRequest_numOfAccesses, sameAddressRequest_numOfAddresses,
      sameAddressRequest_numOfAccesses2, sameAddressRequest_numOfAddresses2,
      sameContentRequest_numOfAccesses, sameContentRequest_numOfContent,
      approxContentRequest_numOfAccesses, approxContentRequest_numOfContent
  );
  fprintf(resOutFile_ch, 
    ",addr,%u,%u,%u,%u,%u,%u,content,%u,%u,%u,%u,%u,%u,approx,%u,%u,%u,%u,%u,%u", 
      sameAddressAtSameClusterAccessCountTracking, sameAddressAtOtherClusterAccessCountTracking, sameAddressAtBothClustersAccessCountTracking, sameAddressAtEitherClusterAccessCountTracking, sameAddressAtEitherClusterAccessCountTracking_200, sameAddressAtEitherClusterAccessCountTracking_100,
      sameContentAtSameClusterAccessCountTracking, sameContentAtOtherClusterAccessCountTracking, sameContentAtBothClustersAccessCountTracking, sameContentAtEitherClusterAccessCountTracking, sameContentAtEitherClusterAccessCountTracking_200, sameContentAtEitherClusterAccessCountTracking_100,
      approxContentAtSameClusterAccessCountTracking, approxContentAtOtherClusterAccessCountTracking, approxContentAtBothClustersAccessCountTracking, approxContentAtEitherClusterAccessCountTracking, approxContentAtEitherClusterAccessCountTracking_200, approxContentAtEitherClusterAccessCountTracking_100
  );
  fprintf(resOutFile_ch, 
    ",addr,%u,%u,%u,content,%u,%u,%u,approx,%u,%u,%u,tracing,%u,%u,%u,%u,_,%u,%u,%u,%u", 
      repeatedAccessAddresses.size(), repeatedAccessAddresses_200.size(), repeatedAccessAddresses_100.size(),
      repeatedAccessAddresses_ExactContent.size(), repeatedAccessAddresses_ExactContent_200.size(), repeatedAccessAddresses_ExactContent_100.size(),
      repeatedAccessAddresses_ApproxContent.size(), repeatedAccessAddresses_ApproxContent_200.size(), repeatedAccessAddresses_ApproxContent_100.size(),
      contentOfAddressTracing.size(), addressWithChangedExactContentCount, approxContentOfAddressTracing.size(), addressWithChangedApproxContentCount,
      numOfDistinctContent, numOfDistinctContent2, numOfDistinctContent_approx, numOfDistinctContent_approx2
  );
  fprintf(resOutFile_ch, 
    ",_,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", 
      numOfAddressesWithAlias,
      numOfAddressesWithoutAlias,
      numOfAddressesWithAlias_approx,
      numOfAddressesWithoutAlias_approx,
      numOfAddressesWithAlias_approx_0,
      numOfAddressesWithoutAlias_approx_0,
      numOfAddressesWithAlias_approx_1,
      numOfAddressesWithoutAlias_approx_1,
      numOfAddressesWithAlias_approx_2,
      numOfAddressesWithoutAlias_approx_2
  );
  fclose(resOutFile_ch);
//// end characterization

////// FILE *resOutFile_cm = fopen("testNoCLUT_cluster_mid_.csv", "a");

  // FILE *resOutFile_c;// = fopen("testNoCLUT_cluster_.csv", "a");
  // fstream resOutFileStream_c;
  // unordered_map<unsigned long long, LUTRecord >::iterator itrt_cluster;
  // std::set<unsigned>::iterator itrt2_cluster;
  // std::set< pair < unsigned, long long unsigned > >::iterator itrt3_cluster;
  
  // std::string filePath;
  
  // // resOutFile_c = fopen("testNoCLUT_cluster_final_.csv", "a");
  // for (unsigned idx = 0; idx < _traffic_manager->NoCLUT_cluster.size(); idx++ ) {
  //   // resOutFile_c = fopen("testNoCLUT_cluster_.csv", "a");
  //   // if ( getDeviceID(idx) < _n_shader ) {
  //   //   fprintf(resOutFile_c, "cluster node# %03u,%u,%u\n", idx, _traffic_manager->NoCLUT_cluster.size(), (_n_shader + _n_mem));
  //   // } else {
  //   //   fprintf(resOutFile_c, "mem node# %03u,%u,%u\n", idx, _traffic_manager->NoCLUT_cluster.size(), (_n_shader + _n_mem));
  //   // }
  //   // fclose(resOutFile_c);

  //     // for (itrt_cluster = _traffic_manager->NoCLUT_cluster[idx].begin(); itrt_cluster != _traffic_manager->NoCLUT_cluster[idx].end(); itrt_cluster++) {
  //     //   // resOutFile_c = fopen("testNoCLUT_cluster_.csv", "a");
        
  //     //   fprintf(resOutFile_c, "0x%08llx,",
  //     //     (*itrt_cluster).first
  //     //   );
        
  //     //   for (itrt2_cluster = (*itrt_cluster).second._icntIDs.begin(); itrt2_cluster != (*itrt_cluster).second._icntIDs.end(); itrt2_cluster++) {
  //     //     fprintf(resOutFile_c, "%u,",
  //     //       (*itrt2_cluster)
  //     //     );
  //     //   }
  //     //   fprintf(resOutFile_c, "\n");

  //     //   // fflush(resOutFile_c);
  //     //   // fclose(resOutFile_c);
  //     // }
    
  //   if ( getDeviceID(idx) >= _n_shader ) {
  //     filePath = "testNoCLUT_cluster_final_" + to_string(idx) + "_.csv";
      
  //     // resOutFile_c = fopen(filePath.c_str(), "a");
  //     resOutFileStream_c.open(filePath.c_str(), ios::app);
    
  //     for (itrt_cluster = _traffic_manager->NoCLUT_cluster[idx].begin(); 
  //           itrt_cluster != _traffic_manager->NoCLUT_cluster[idx].end(); 
  //           itrt_cluster++) {
  //       // resOutFile_c = fopen(filePath.c_str(), "a");
        
  //       // fprintf(resOutFile_c, "0x%08llx,",
  //       //   (*itrt_cluster).first
  //       // );
  //       resOutFileStream_c << "0x" << to_string((*itrt_cluster).first) ;
        
  //       // for (unsigned idx3 = 0; idx3 < (*itrt_cluster).second._records.size(); idx3++ ) {
  //       //   fprintf(resOutFile_c, "%u_%llu,",
  //       //     (*itrt_cluster).second._records[idx3].first,
  //       //     (*itrt_cluster).second._records[idx3].second
  //       //   );
  //       // }
  //       for ( itrt3_cluster = (*itrt_cluster).second._records.begin();
  //             itrt3_cluster != (*itrt_cluster).second._records.end();
  //             itrt3_cluster++ ) {
  //         // fprintf(resOutFile_c, "%u_%llu,",
  //         //   (*itrt3_cluster).first,
  //         //   (*itrt3_cluster).second
  //         // );
  //         resOutFileStream_c << "," << to_string((*itrt3_cluster).first) 
  //                             << "_" << to_string((*itrt3_cluster).second);

  //       }  
  //       // fprintf(resOutFile_c, "\n");
  //       resOutFileStream_c << "\n";

  //       // fflush(resOutFile_c);
  //       // fclose(resOutFile_c);
  //     }
      
  //     // fclose(resOutFile_c);
  //     resOutFileStream_c.close();
  //   }
  // }
// // fclose(resOutFile_c);
}


// void InterconnectInterface::printLUT2(FILE *resOutFile_mP) {
void InterconnectInterface::printLUT2() {

//   FILE *resOutFile_mP;// = fopen("testNoCLUT_memP_final_.csv", "a"); 
//   unordered_map<std::string, std::set<unsigned > >::iterator itrt_mem;
//   std::set<unsigned>::iterator itrt2_mem;
// resOutFile_mP = fopen("testNoCLUT_memP_.csv", "a"); 
//   for (unsigned idx = 0; idx < (_n_shader + _n_mem); idx++ ) {
//     // resOutFile_mP = fopen("testNoCLUT_memP_.csv", "a"); 
//     if ( getDeviceID(idx) >= _n_shader ) {
//       fprintf(resOutFile_mP, "mem 2 node# %03u,%u,%u\n", idx, _traffic_manager->NoCLUT_memPart.size(), (_n_shader + _n_mem));
//     } else {
//       fprintf(resOutFile_mP, "cluster 2 node# %03u,%u,%u\n", idx, _traffic_manager->NoCLUT_memPart.size(), (_n_shader + _n_mem));
//     }
//     // fclose(resOutFile_mP);

//       for (itrt_mem = _traffic_manager->NoCLUT_memPart[idx].begin(); itrt_mem != _traffic_manager->NoCLUT_memPart[idx].end(); itrt_mem++ ) {
//         // resOutFile_mP = fopen("testNoCLUT_memP_.csv", "a"); 

//         fprintf(resOutFile_mP, "%s,",
//           (*itrt_mem).first.c_str()
//         );

//         for (itrt2_mem = (*itrt_mem).second.begin(); itrt2_mem != (*itrt_mem).second.end(); itrt2_mem++ ) {
//           fprintf(resOutFile_mP, "%03u,",
//             (*itrt2_mem)
//           );
//         }
//         fprintf(resOutFile_mP, "\n");
        
//         // fflush(resOutFile_mP);
//         // fclose(resOutFile_mP);
//       }
    
//   }
// fclose(resOutFile_mP);
  


// ////

//   // FILE *resOutFile = fopen("testNoCLUT_.csv", "a");
//   // unordered_map<std::string, std::set<unsigned > >::iterator itrt;
//   // std::set<unsigned>::iterator itrt2;
//   // for (itrt = _traffic_manager->NoCLUT.begin(); itrt != _traffic_manager->NoCLUT.end(); itrt++) {
//   //     fprintf(resOutFile, "%s,",
//   //       (*itrt).first.c_str()
//   //     );

//   //     for (itrt2 = (*itrt).second.begin(); itrt2 != (*itrt).second.end(); itrt2++) {
//   //       fprintf(resOutFile, "%u,",
//   //         (*itrt2)
//   //       );
        
//   //     }
//   //     fprintf(resOutFile, "\n");
//   // }

//   // fclose(resOutFile); 
  
//   // ////
  
//   // FILE *resOutFile_a = fopen("testNoCLUT_addr_.csv", "a");
//   // unordered_map<unsigned long long, std::set<unsigned > >::iterator itrt_a;
//   // std::set<unsigned>::iterator itrt2_a;
//   // for (itrt_a = _traffic_manager->NoCLUT_a.begin(); itrt_a != _traffic_manager->NoCLUT_a.end(); itrt_a++) {
//   //     fprintf(resOutFile_a, "0x%08llx,",
//   //       (*itrt_a).first
//   //     );
      
//   //     for (itrt2_a = (*itrt_a).second.begin(); itrt2_a != (*itrt_a).second.end(); itrt2_a++) {
//   //       fprintf(resOutFile_a, "%u,",
//   //         (*itrt2_a)
//   //       );
        
//   //     }
//   //     fprintf(resOutFile_a, "\n");
//   // }

//   // fclose(resOutFile_a); 
}
////


void InterconnectInterface::Advance(long long unsigned curCycle)
{
  // _traffic_manager->_Step(); ////
  _traffic_manager->_Step(curCycle); // Khoa
  
  // FILE *resOutFile_destCount = fopen("testChangePacketDestCount_.csv", "a");  
  // fprintf(resOutFile_destCount, "%llu,%lld,%u,%u,_,%u,%u,%u,_,%u,%u,%u,%u,%u,%u,%u,%u\n", 
  //   curCycle,
  //   _traffic_manager->getTime(),
  //   redirectedCount,
  //   redirectedPackets.size(),
  //   pushCount,
  //   popCount,
  //   popCount_noData,
  //   readRequestPushCount,
  //   writeRequestPushCount,
  //   readReplyPushCount,
  //   writeReplyPushCount,
  //   readRequestPopCount,
  //   writeRequestPopCount,
  //   readReplyPopCount,
  //   writeReplyPopCount
  //   );
  // fclose(resOutFile_destCount);
}

bool InterconnectInterface::Busy() const
{
  bool busy = !_traffic_manager->_total_in_flight_flits[0].empty();
  if (!busy) {
    for (int s = 0; s < _subnets; ++s) {
      for (unsigned n = 0; n < _n_shader+_n_mem; ++n) {
        //FIXME: if this cannot make sure _partial_packets is empty
        assert(_traffic_manager->_input_queue[s][n][0].empty());
      }
    }
  }
  else
    return true;
  for (int s = 0; s < _subnets; ++s) {
    for (unsigned n=0; n < (_n_shader+_n_mem); ++n) {
      for (int vc=0; vc<_vcs; ++vc) {
        if (_boundary_buffer[s][n][vc].HasPacket() ) {
          return true;
        }
      }
    }
  }
  return false;
}

bool InterconnectInterface::HasBuffer(unsigned deviceID, unsigned int size, unsigned subnet) const // Khoa
// bool InterconnectInterface::HasBuffer(unsigned deviceID, unsigned int size) const
{
  bool has_buffer = false;
  unsigned int n_flits = size / _flit_size + ((size % _flit_size)? 1:0);
  int icntID = _node_map.find(deviceID)->second;

  // has_buffer = _traffic_manager->_input_queue[0][icntID][0].size() + n_flits <= _input_buffer_capacity;

  // if ((_subnets>1) && deviceID >= _n_shader) { // deviceID is memory node
  //   // only check reply network
  //   has_buffer = _traffic_manager->_input_queue[1][icntID][0].size() + n_flits <= _input_buffer_capacity;
  // } else { // deviceID shows non-memory node
  //   // check either request or reply network
  //   has_buffer = _traffic_manager->_input_queue[subnet][icntID][0].size() + n_flits <= _input_buffer_capacity;
  // }

  if ( _subnets > 1 ) {
    has_buffer = (_traffic_manager->_input_queue[subnet][icntID][0].size() + n_flits) <= _input_buffer_capacity;
  } else {
    has_buffer = (_traffic_manager->_input_queue[0][icntID][0].size() + n_flits) <= _input_buffer_capacity;
  }

  return has_buffer;
}

bool InterconnectInterface::HasVCBuffer(unsigned deviceID, unsigned int size, unsigned subnet, unsigned mode) const // Khoa
// bool InterconnectInterface::HasBuffer(unsigned deviceID, unsigned int size) const
{
  bool has_buffer = false;
  
  unsigned int n_flits = size / _flit_size + ((size % _flit_size)? 1:0);
  int icntID = _node_map.find(deviceID)->second;

  unsigned vcBegin;
  unsigned vcEnd;

  if (subnet == 0) {
    if (mode == 0) {
      vcBegin = _icnt_config->GetInt("read_request_begin_vc");
      vcEnd = _icnt_config->GetInt("read_request_end_vc");
    } else if (mode == 1) {
      vcBegin = _icnt_config->GetInt("write_request_begin_vc");
      vcEnd = _icnt_config->GetInt("write_request_end_vc");
    } else {
      // vcBegin = 0;
      // vcEnd = _icnt_config->GetInt("num_vcs") - 1;
      vcBegin = _icnt_config->GetInt("read_request_begin_vc");
      vcEnd = _icnt_config->GetInt("write_request_end_vc");
    }
  } else if (subnet == 1) {
    if (mode == 0) {
      vcBegin = _icnt_config->GetInt("read_reply_begin_vc");
      vcEnd = _icnt_config->GetInt("read_reply_end_vc");
    } else if (mode == 1) {
      vcBegin = _icnt_config->GetInt("write_reply_begin_vc");
      vcEnd = _icnt_config->GetInt("write_reply_end_vc");
    } else {
      // vcBegin = 0;
      // vcEnd = _icnt_config->GetInt("num_vcs") - 1;
      vcBegin = _icnt_config->GetInt("read_reply_begin_vc");
      vcEnd = _icnt_config->GetInt("write_reply_end_vc");
    }
  }
  
  int freeSlot = 0;
  for (unsigned vc = vcBegin; vc <= vcEnd; vc++) {
    BufferState * curBufState = _traffic_manager->_buf_states[icntID][subnet];
    freeSlot = curBufState->LimitFor(vc) - curBufState->OccupancyFor(vc);
    // has_buffer = (freeSlot > 0);
    has_buffer = freeSlot > (_traffic_manager->_input_queue[subnet][icntID][0].size() + n_flits); 
    if (has_buffer) {
      break;
    } 
  }

  return has_buffer;
}

void InterconnectInterface::DisplayStats() const
{
  _traffic_manager->UpdateStats();
  _traffic_manager->DisplayStats();
}

unsigned InterconnectInterface::GetFlitSize() const
{
  return _flit_size;
}

void InterconnectInterface::DisplayOverallStats() const
{
  // hack: booksim2 use _drain_time and calculate delta time based on it, but we don't, change this if you have a better idea
  _traffic_manager->_drain_time = _traffic_manager->_time;
  // hack: also _total_sims equals to number of kernel calls
  _traffic_manager->_total_sims += 1;

  _traffic_manager->_UpdateOverallStats();
  _traffic_manager->DisplayOverallStats();
  if(_traffic_manager->_print_csv_results) {
    _traffic_manager->DisplayOverallStatsCSV();
  }
}

void InterconnectInterface::DisplayState(FILE *fp) const
{
  fprintf(fp, "GPGPU-Sim uArch: ICNT:Display State: Under implementation\n");
//  fprintf(fp,"GPGPU-Sim uArch: interconnect busy state\n");

//  for (unsigned i=0; i<net_c;i++) {
//    if (traffic[i]->_measured_in_flight)
//      fprintf(fp,"   Network %u has %u _measured_in_flight\n", i, traffic[i]->_measured_in_flight );
//  }
//
//  for (unsigned i=0 ;i<(_n_shader+_n_mem);i++ ) {
//    if( !traffic[0]->_partial_packets[i] [0].empty() )
//      fprintf(fp,"   Network 0 has nonempty _partial_packets[%u][0]\n", i);
//    if ( doub_net && !traffic[1]->_partial_packets[i] [0].empty() )
//      fprintf(fp,"   Network 1 has nonempty _partial_packets[%u][0]\n", i);
//    for (unsigned j=0;j<g_num_vcs;j++ ) {
//      if( !ejection_buf[i][j].empty() )
//        fprintf(fp,"   ejection_buf[%u][%u] is non-empty\n", i, j);
//      if( clock_boundary_buf[i][j].has_packet() )
//        fprintf(fp,"   clock_boundary_buf[%u][%u] has packet\n", i, j );
//    }
//  }
}

void InterconnectInterface::Transfer2BoundaryBuffer(int subnet, int output)
{
  Flit* flit;
  int vc;
  for (vc=0; vc<_vcs;vc++) {

    if ( !_ejection_buffer[subnet][output][vc].empty() && _boundary_buffer[subnet][output][vc].Size() < _boundary_buffer_capacity ) {
      flit = _ejection_buffer[subnet][output][vc].front();
      assert(flit);

      _ejection_buffer[subnet][output][vc].pop();
      ////
      mem_fetch* mft = static_cast<mem_fetch* >(flit->data);
      mft->time_RequestReceived = _traffic_manager->getTime();
      ////
      _boundary_buffer[subnet][output][vc].PushFlitData( flit->data, flit->tail);

      _ejected_flit_queue[subnet][output].push(flit); //indicate this flit is already popped from ejection buffer and ready for credit return

      if ( flit->head ) {
        assert (flit->dest == output);

////
// if (flit->redirectedFlag) {
//   FILE *resOutFile_boundary = fopen("testBoundaryBuffer_.csv", "a");  
//   fprintf(resOutFile_boundary, "%u,%03d,%03d,%03d|%03d\n", 
//     flit->pid,
//     output,
//     flit->src,
//     flit->destMC,
//     flit->dest
//     );

//   // for (int idx = 0; idx < flit->redirectNodes.size(); idx++ ) {
//   //   fprintf(resOutFile_boundary, "%u_%u|", 
//   //     flit->redirectNodes[idx],
//   //     flit->redirectDests[idx]
//   //     );
//   // }

//   fclose(resOutFile_boundary);
// }
////
// if (flit->pid == 18) {
//   FILE *resOutFile_boundary = fopen("testTr2BoundaryBuffer_.csv", "a");  
//   fprintf(resOutFile_boundary, ",%u,%03d,%03d,%03d|%03d,%d,%d\n", 
//     flit->pid,
//     output,
//     flit->src,
//     flit->destMC,
//     flit->dest,
//     subnet,
//     vc
//     );
//   fclose(resOutFile_boundary);
// } 
////
      }
    }
  }
}

void InterconnectInterface::WriteOutBuffer(int subnet, int output_icntID, Flit*  flit )
{
  int vc = flit->vc;
  assert (_ejection_buffer[subnet][output_icntID][vc].size() < _ejection_buffer_capacity);
  _ejection_buffer[subnet][output_icntID][vc].push(flit);

////
// if (flit->pid == 18) {
//   FILE *resOutFile_boundary = fopen("testWriteEjectionBuffer_.csv", "a");  
//   fprintf(resOutFile_boundary, ",%u,%03d,%03d,%03d|%03d,%d,%d\n", 
//     flit->pid,
//     output_icntID,
//     flit->src,
//     flit->destMC,
//     flit->dest,
//     subnet,
//     vc
//     );
//   fclose(resOutFile_boundary);
// } 
/////

}

int InterconnectInterface::GetIcntTime() const
{
  return _traffic_manager->getTime();
}

Stats* InterconnectInterface::GetIcntStats(const string &name) const
{
  return _traffic_manager->getStats(name);
}

Flit* InterconnectInterface::GetEjectedFlit(int subnet, int node)
{
  Flit* flit = NULL;
  if (!_ejected_flit_queue[subnet][node].empty()) {
    flit = _ejected_flit_queue[subnet][node].front();
    _ejected_flit_queue[subnet][node].pop();
  }
  return flit;
}

void InterconnectInterface::_CreateBuffer()
{
  unsigned nodes = _net[0]->NumNodes();

  _boundary_buffer.resize(_subnets);
  _ejection_buffer.resize(_subnets);
  _round_robin_turn.resize(_subnets);
  _ejected_flit_queue.resize(_subnets);

  for (int subnet = 0; subnet < _subnets; ++subnet) {
    _ejection_buffer[subnet].resize(nodes);
    _boundary_buffer[subnet].resize(nodes);
    _round_robin_turn[subnet].resize(nodes);
    _ejected_flit_queue[subnet].resize(nodes);

    for (unsigned node=0;node < nodes;++node){
      _ejection_buffer[subnet][node].resize(_vcs);
      _boundary_buffer[subnet][node].resize(_vcs);
    }
  }
}

void InterconnectInterface::_CreateNodeMap(unsigned n_shader, unsigned n_mem, unsigned n_node, int use_map)
{
  if (use_map) {
    // The (<SM, Memory>, Memory Location Vector) map
    map<pair<unsigned,unsigned>, vector<unsigned> > preset_memory_map;

    {
      // preset memory and shader map, optimized for mesh
      // good for 8 SMs and 8 memory ports, the map is as follows:
      // +--+--+--+--+
      // |C0|M0|C1|M1|
      // +--+--+--+--+
      // |M2|C2|M3|C3|
      // +--+--+--+--+
      // |C4|M4|C5|M5|
      // +--+--+--+--+
      // |M6|C6|M7|C7|
      // +--+--+--+--+
    
      unsigned memory_node[] = {1, 3, 4, 6, 9, 11, 12, 14};
      preset_memory_map[make_pair(8,8)] = vector<unsigned>(memory_node, memory_node+8);
    }

    
    

    // // good for 56 SMs and 8 memory cores
    // {
    //   unsigned memory_node[] = {3, 15, 17, 29, 36, 47, 49, 61};
    //   preset_memory_map[make_pair(56,8)] = vector<unsigned>(memory_node, memory_node+sizeof(memory_node)/sizeof(unsigned));
    // }

    {
      // good for 110 SMs and 11 memory cores
      unsigned memory_node[] = {12, 20, 25, 28, 57, 60, 63, 92, 95,100,108};
      preset_memory_map[make_pair(110, 11)] = vector<unsigned>(memory_node, memory_node+sizeof(memory_node)/sizeof(unsigned));
    }

    // Khoa
    if(use_map == 1) {

      {
        // good for 28 SMs and 8 memory ports
        unsigned memory_node[] = {3, 7, 10, 12, 23, 25, 28, 32};
        preset_memory_map[make_pair(28,8)] = vector<unsigned>(memory_node, memory_node+8);
      }

      {
        unsigned memory_node[] = { 1, 3, 5, 7, 
                                  8, 10, 12, 14, 
                                  17, 19, 21, 23,
                                  24, 26, 28, 30,
                                  33, 35, 37, 39,
                                  40, 42, 44, 46,
                                  49, 51, 53, 55,
                                  56, 58, 60, 62 };
        preset_memory_map[make_pair(32, 32)] = vector<unsigned>(memory_node, memory_node+sizeof(memory_node)/sizeof(unsigned));
      }
      
      {
        unsigned memory_node[] = { 9, 10, 11, 12, 13, 14, 
                                  18, 20, 22,
                                  25, 27, 29,
                                  34, 36, 38,
                                  41, 43, 45,
                                  49, 50, 51, 52, 53, 54 };
        preset_memory_map[make_pair(40, 24)] = vector<unsigned>(memory_node, memory_node+sizeof(memory_node)/sizeof(unsigned));
      }

      {
        unsigned memory_node[] = { 10, 12, 14, 
                                  17, 19, 21, 
                                  26, 27, 28, 30, 
                                  33, 35, 36, 37, 
                                  42, 44, 46, 
                                  49, 51, 53 };
        preset_memory_map[make_pair(44, 20)] = vector<unsigned>(memory_node, memory_node+sizeof(memory_node)/sizeof(unsigned));
      }

      {
        // unsigned memory_node[] = { 0, 1, 2, 3, 4, 5, 6, 7, 56, 57, 58, 59, 60, 61, 62, 63 };
        // unsigned memory_node[] = { 9, 11, 13, 15, 
        //                           24, 26, 28, 30, 
        //                           41, 43, 45, 47, 
        //                           56, 58, 60, 62 };
        // preset_memory_map[make_pair(48, 16)] = vector<unsigned>(memory_node, memory_node+sizeof(memory_node)/sizeof(unsigned));
        unsigned memory_node[] = { 10, 13, 17, 19, 20, 22, 26, 29, 34, 37, 41, 43, 44, 46, 50, 53 };
        preset_memory_map[make_pair(48, 16)] = vector<unsigned>(memory_node, memory_node+sizeof(memory_node)/sizeof(unsigned));
      }

      {
        // unsigned memory_node[] = { 17, 19, 21, 23, 41, 43, 45, 47 };
        unsigned memory_node[] = { 10, 13, 17, 22, 41, 46, 50, 53 };
        preset_memory_map[make_pair(56, 8)] = vector<unsigned>(memory_node, memory_node+sizeof(memory_node)/sizeof(unsigned));
      }
    }

    const vector<int> config_memory_node(_icnt_config->GetIntArray("memory_node_map"));
    if (!config_memory_node.empty()) {
      if (config_memory_node.size() != _n_mem) {
        cerr << "Number of memory nodes in memory_node_map should equal to memory ports" << endl;
        assert( config_memory_node.size() == _n_mem);
      }
      vector<unsigned> t_memory_node(config_memory_node.size());
      copy(config_memory_node.begin(), config_memory_node.end(), t_memory_node.begin());
      preset_memory_map[make_pair(_n_shader, _n_mem)] = t_memory_node;
    }
    
    const vector<unsigned> &memory_node = preset_memory_map[make_pair(_n_shader, _n_mem)];
    if (memory_node.empty()) {
      cerr<<"ERROR!!! NO MAPPING IMPLEMENTED YET FOR THIS CONFIG"<<endl;
      assert(0);
    }

    // create node map
    unsigned next_node = 0;
    unsigned memory_node_index = 0;
    for (unsigned i = 0; i < n_shader; ++i) {
      while (next_node == memory_node[memory_node_index]) {
        next_node += 1;
        memory_node_index += 1;
      }
      _node_map[i] = next_node;
      next_node += 1;
    }
    for (unsigned i = n_shader; i < n_shader+n_mem; ++i) {
      _node_map[i] = memory_node[i-n_shader];
    }
  } else { //not use preset map
    for (unsigned i=0;i<n_node;i++) {
      _node_map[i]=i;
    }
  }

  for (unsigned i = 0; i < n_node ; i++) {
    for (unsigned j = 0; j< n_node ; j++) {
      if ( _node_map[j] == i ) {
        _reverse_node_map[i]=j;
        break;
      }
    }
  }

  //FIXME: should compatible with non-square number
  _DisplayMap((int) sqrt(n_node), n_node);

}

void InterconnectInterface::_DisplayMap(int dim,int count)
{
  cout << "GPGPU-Sim uArch: interconnect node map (shaderID+MemID to icntID)" << endl;
  cout << "GPGPU-Sim uArch: Memory nodes ID start from index: " << _n_shader << endl;
  cout << "GPGPU-Sim uArch: ";
  for (int i = 0;i < count; i++) {
    cout << setw(4) << _node_map[i];
    if ((i+1)%dim == 0 && i != count-1)
      cout << endl << "GPGPU-Sim uArch: ";
  }
  cout << endl;

  cout << "GPGPU-Sim uArch: interconnect node reverse map (icntID to shaderID+MemID)" << endl;
  cout << "GPGPU-Sim uArch: Memory nodes start from ID: " << _n_shader << endl;
  cout << "GPGPU-Sim uArch: ";
  for (int i = 0;i < count; i++) {
    cout << setw(4) << _reverse_node_map[i];
    if ((i+1)%dim == 0 && i != count-1)
      cout << endl << "GPGPU-Sim uArch: ";
  }
  cout << endl;
}

void* InterconnectInterface::_BoundaryBufferItem::PopPacket()
{
  assert (_packet_n);
  void * data = NULL;
  void * flit_data = _buffer.front();
  while (data == NULL) {
    assert(flit_data == _buffer.front()); //all flits must belong to the same packet
    if (_tail_flag.front()) {
      data = _buffer.front();
      _packet_n--;
    }
    _buffer.pop();
    _tail_flag.pop();
  }
  return data;
}

void* InterconnectInterface::_BoundaryBufferItem::TopPacket() const
{
  assert (_packet_n);
  void* data = NULL;
  void* temp_d = _buffer.front();
  while (data==NULL) {
    if (_tail_flag.front()) {
      data = _buffer.front();
    }
    assert(temp_d == _buffer.front()); //all flits must belong to the same packet
  }
  return data;

}

void InterconnectInterface::_BoundaryBufferItem::PushFlitData(void* data,bool is_tail)
{
  _buffer.push(data);
  _tail_flag.push(is_tail);
  if (is_tail) {
    _packet_n++;
  }
}
