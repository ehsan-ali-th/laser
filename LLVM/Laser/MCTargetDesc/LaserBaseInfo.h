//===-- LaserBaseInfo.h - Top level definitions for LASER MC ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone helper functions and enum definitions for
// the Cpu0 target useful for the compiler back-end and the MC libraries.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_LASER_MCTARGETDESC_LASERBASEINFO_H
#define LLVM_LIB_TARGET_LASER_MCTARGETDESC_LASERBASEINFO_H

#include "MCTargetDesc/LaserMCTargetDesc.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/Support/DataTypes.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// LaserII - This namespace holds all of the target specific flags that
/// instruction info tracks.
//@LaserII
namespace LaserII {
  /// Target Operand Flag enum.
  enum TOF {
    //===------------------------------------------------------------------===//
    // Laser Specific MachineOperand flags.

    MO_NO_FLAG,
    
    // Represents a 16-bit address
    MO_CALL_FLAG,

    // Represents an 11-bit address
    MO_JMP_FLAG,

    // Represents 16-bit global address
    MO_ABS

  }; // enum TOF {

  enum {
    //===------------------------------------------------------------------===//
    // Instruction encodings.  These are the standard/most common forms for
    // Cpu0 instructions.
    //

    // Pseudo - This represents an instruction that is a pseudo instruction
    // or one that has not been implemented yet.  It is illegal to code generate
    // it, but tolerated for intermediate implementation stages.
    LaserPseudo   = 0,

    /// F1 - This form is for instructions of the format 1.
    F1  = 1,
    /// F2 - This form is for instructions of the format 2.
    F2  = 2,
    /// F3 - This form is for instructions of the format 3.
    F3  = 3,
    /// FJ - This form is for instructions of the format 4.
    FJ = 4,
    /// FLong1 - This form is for instructions of the format long.
    F_long_1 = 5,

    FormMask = 15
  };

}

}

#endif
