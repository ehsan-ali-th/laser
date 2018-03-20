//===-- LaserTargetInfo.cpp - Laser Target Implementation -----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "Laser.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

namespace llvm {
Target &getTheLaserTarget() {
  static Target TheLaserTarget;
  return TheLaserTarget;
}
} // namespace llvm

//Target llvm::TheLaserTarget;

extern "C" void LLVMInitializeLaserTargetInfo() {
  RegisterTarget<Triple::laser> Z(getTheLaserTarget(), "laser", "Laser",
                                                   "Laser");
}
