//===-- llvm/Target/LaserTargetObjectFile.h - Laser Object Info ---*- C++ -*-===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_LASER_LASERTARGETOBJECTFILE_H
#define LLVM_LIB_TARGET_LASER_LASERTARGETOBJECTFILE_H

#include "LaserTargetMachine.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {

  class MCContext;
  class TargetMachine;

  class LaserELFTargetObjectFile : public TargetLoweringObjectFileELF {
    MCSection *SmallDataSection;
    MCSection *SmallBSSSection;
    const LaserTargetMachine *TM;

  public:

    void Initialize(MCContext &Ctx, const TargetMachine &TM) override;

    // LaserELFTargetObjectFile() :
    //   TargetLoweringObjectFileELF()
    // {}

    MCSection *SelectSectionForGlobal(const GlobalObject *GO, SectionKind Kind,
                                    const TargetMachine &TM) const override;
  };
} // end namespace llvm

#endif
