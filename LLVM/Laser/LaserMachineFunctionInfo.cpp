//===-- LaserMachineFunctionInfo.cpp - Laser Machine Function Info --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "LaserMachineFunctionInfo.h"
#include "LaserInstrInfo.h"
#include "LaserSubtarget.h"

#include "llvm/IR/Function.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

using namespace llvm;

bool FixGlobalBaseReg;

LaserMachineFunctionInfo::~LaserMachineFunctionInfo() {}

void LaserMachineFunctionInfo::anchor() { }

MachinePointerInfo LaserMachineFunctionInfo::callPtrInfo(const char *ES) {
  return MachinePointerInfo(MF.getPSVManager().getExternalSymbolCallEntry(ES));
}

MachinePointerInfo LaserMachineFunctionInfo::callPtrInfo(const GlobalValue *GV) {
  return MachinePointerInfo(MF.getPSVManager().getGlobalValueCallEntry(GV));
}

unsigned LaserMachineFunctionInfo::getGlobalBaseReg() {
  return GlobalBaseReg = LASER::GP;
}

