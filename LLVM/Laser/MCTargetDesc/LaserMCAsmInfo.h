//===-- LaserMCAsmInfo.h - Laser asm properties ----------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the LaserMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_LASER_MCTARGETDESC_LASERMCASMINFO_H
#define LLVM_LIB_TARGET_LASER_MCTARGETDESC_LASERMCASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {
class Triple;

class LaserELFMCAsmInfo : public MCAsmInfoELF {
  void anchor() override;

public:
  explicit LaserELFMCAsmInfo(const Triple &TheTriple);
};
  
} // namespace llvm

#endif
