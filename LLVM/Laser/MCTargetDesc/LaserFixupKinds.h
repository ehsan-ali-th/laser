//===-- LaserFixupKinds.h - Laser Specific Fixup Entries ----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_LASER_MCTARGETDESC_LASERFIXUPKINDS_H
#define LLVM_LIB_TARGET_LASER_MCTARGETDESC_LASERFIXUPKINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
namespace Laser {
  // Although most of the current fixup types reflect a unique relocation
  // one can have multiple fixup types for a given relocation and thus need
  // to be uniquely named.
  //
  // This table *must* be in the save order of
  // MCFixupKindInfo Infos[Laser::NumTargetFixupKinds]
  // in LaserAsmBackend.cpp.
  //@Fixups {
  enum Fixups {
    //@ Pure upper 16 bit fixup resulting in - R_LASER_CALL16.
    fixup_Laser_CALL16 = FirstTargetFixupKind,

    // PC relative branch fixup resulting in - R_LASER_PC11.
    // Laser JMP
    fixup_Laser_PC11,
    
    // Global variable fixup resulting in - R_LASER_GV16.
    fixup_Laser_GV16,

    // Marker
    LastTargetFixupKind,
    NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
  };
  //@Fixups }
} // namespace Laser
} // namespace llvm

#endif // LLVM_LASER_LASERFIXUPKINDS_H
