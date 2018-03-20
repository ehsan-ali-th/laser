//===-- LaserTargetStreamer.h - Laser Target Streamer ----------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_LASER_LASERTARGETSTREAMER_H
#define LLVM_LIB_TARGET_LASER_LASERTARGETSTREAMER_H

#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"

namespace llvm {

class LaserTargetStreamer : public MCTargetStreamer {
public:
  LaserTargetStreamer(MCStreamer &S);
};

// This part is for ascii assembly output
class LaserTargetAsmStreamer : public LaserTargetStreamer {
  formatted_raw_ostream &OS;

public:
  LaserTargetAsmStreamer(MCStreamer &S, formatted_raw_ostream &OS);
};

}
#endif
