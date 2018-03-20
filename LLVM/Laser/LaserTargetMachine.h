//===-- LaserTargetMachine.h - Define TargetMachine for Laser ---*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the Laser specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_LASER_LASERTARGETMACHINE_H
#define LLVM_LIB_TARGET_LASER_LASERTARGETMACHINE_H

//#include "MCTargetDesc/LaserABIInfo.h"
#include "LaserInstrInfo.h"
#include "LaserSubtarget.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

  class LaserTargetMachine : public LLVMTargetMachine {
    std::unique_ptr<TargetLoweringObjectFile> TLOF;
    // Selected ABI
    //    LaserABIInfo ABI;
    LaserSubtarget DefaultSubtarget;
    bool isLittle;
    mutable StringMap<std::unique_ptr<LaserSubtarget>> SubtargetMap;

  public:
    LaserTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
		       StringRef FeatureString, const TargetOptions &Options,
		       Optional<Reloc::Model> RM, Optional<CodeModel::Model> CodeModel,
		       CodeGenOpt::Level OptLevel, bool JIT
		       );

    ~LaserTargetMachine() override;

    const LaserSubtarget *getSubtargetImpl() const { return &DefaultSubtarget; }
    const LaserSubtarget *getSubtargetImpl(const Function &F) const override;

    // Pass Pipeline Configuration
    TargetPassConfig *createPassConfig(PassManagerBase &PM) override;
    TargetLoweringObjectFile *getObjFileLowering() const override {
      return TLOF.get();
    }

    //    const LaserABIInfo &getABI() const { return ABI; }
  };

  /// LaserelTargetMachine - Laser little endian target machine.
  ///
  // class LaserelTargetMachine : public LaserTargetMachine {
  //   virtual void anchor();
  // public:
  //   LaserelTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
  //                      StringRef FS, const TargetOptions &Options,
  //                      Optional<Reloc::Model> RM, CodeModel::Model CM,
  //                      CodeGenOpt::Level OL);
  // };

} // end llvm namespace

#endif
