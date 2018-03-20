//===-- LaserSubtarget.cpp - LASER Subtarget Information ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the LASER specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "LaserSubtarget.h"
#include "LaserMachineFunctionInfo.h"
#include "Laser.h"
#include "LaserRegisterInfo.h"

#include "LaserTargetMachine.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "laser-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "LaserGenSubtargetInfo.inc"

extern bool FixGlobalBaseReg;

void LaserSubtarget::anchor() { }

LaserSubtarget::LaserSubtarget(const Triple &TT, const std::string &CPU,
                               const std::string &FS, bool isLittle, 
			       const LaserTargetMachine &_TM) : 
    LaserGenSubtargetInfo(TT, CPU, FS), 
    IsLittle(isLittle), 
    TM(_TM), 
    TargetTriple(TT),
    TSInfo(),
    InstrInfo(LaserInstrInfo::create(initializeSubtargetDependencies(CPU, FS, TM))),
    FrameLowering(LaserFrameLowering::create(*this)), 
    TLInfo(LaserTargetLowering::create(TM, *this)) 
{}

LaserSubtarget &LaserSubtarget::initializeSubtargetDependencies(StringRef CPU, 
								StringRef FS,
								const TargetMachine &TM) {
  // Determine default and user specified characteristics
  std::string CPUName = CPU;
  if (CPUName.empty())
    CPUName = "Generic";

  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);

  return *this;
}

bool LaserSubtarget::abiUsesSoftFloat() const {
//  return TM->Options.UseSoftFloat;
  return true;
}

// const LaserABIInfo &LaserSubtarget::getABI() const { return TM.getABI(); } 
