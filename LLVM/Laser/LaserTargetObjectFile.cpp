//===-- LaserTargetObjectFile.cpp - Laser Object Files ----------------------===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "LaserTargetObjectFile.h"
#include "MCTargetDesc/LaserMCExpr.h"
#include "llvm/CodeGen/MachineModuleInfoImpls.h"
#include "llvm/CodeGen/TargetLowering.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/BinaryFormat/ELF.h"

using namespace llvm;

static cl::opt<unsigned>
SSThreshold("cpu0-ssection-threshold", cl::Hidden,
            cl::desc("Small data and bss section threshold size (default=8)"),
            cl::init(8));

void LaserELFTargetObjectFile::Initialize(MCContext &Ctx, const TargetMachine &TM){
  TargetLoweringObjectFileELF::Initialize(Ctx, TM);
  InitializeELF(TM.Options.UseInitArray);

  SmallDataSection = getContext().getELFSection(
      ".sdata", ELF::SHT_PROGBITS, ELF::SHF_WRITE | ELF::SHF_ALLOC);

  SmallBSSSection = getContext().getELFSection(".sbss", ELF::SHT_NOBITS,
                                               ELF::SHF_WRITE | ELF::SHF_ALLOC);
  this->TM = &static_cast<const LaserTargetMachine &>(TM);
}


MCSection *
LaserELFTargetObjectFile::SelectSectionForGlobal(const GlobalObject *GO, SectionKind Kind,
					      const TargetMachine &TM) const {

  // Handle Small Section classification here.
  if (Kind.isBSS())
    return SmallBSSSection;
  if (Kind.isData())
    return SmallDataSection;

  // Otherwise, we work the same as ELF.
  return TargetLoweringObjectFileELF::SelectSectionForGlobal(GO, Kind, TM);
}

