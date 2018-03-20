//===-- Laser.h - Top-level interface for Laser representation --*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// Laser back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_LASER_LASER_H
#define LLVM_LIB_TARGET_LASER_LASER_H

#include "LaserCondCode.h"

#include "MCTargetDesc/LaserMCTargetDesc.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetMachine.h"


namespace llvm {
  class LaserTargetMachine;
  class FunctionPass;
  class AsmPrinter;
  class MCInst;
  class MachineInstr;

  void LowerLaserMachineInstrToMCInst(const MachineInstr *MI,
                                      MCInst &OutMI,
                                      AsmPrinter &AP);

  FunctionPass *createLaserISelDag(LaserTargetMachine &TM);

} // end namespace llvm;


#endif
