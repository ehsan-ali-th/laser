//===-- LaserTargetMachine.cpp - Define TargetMachine for Laser -----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

#include "LaserTargetMachine.h"
#include "LaserTargetObjectFile.h"
#include "Laser.h"

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

extern "C" void LLVMInitializeLaserTarget() {
  // Register the target.
  RegisterTargetMachine<LaserTargetMachine> Z(getTheLaserTarget());
}

static StringRef computeDataLayout(const Triple &T) {
  // Laser is Little Indian
  std::string Ret = "";

  Ret += "E"; // Big endian

  Ret += "-m:e"; // ELF name mangling
  // first value is pointer size, and the second value is both ABI 
  //   and preferred alignment.
  Ret += "-p:16:16"; // 16-bit pointers, 16 bit aligned
  // Alignments for 16 bit integers.
  Ret += "-i16:16"; // 16 bit integers, 16 bit aligned
  Ret += "-a:0:16";  // 16 bit alignment of objects of aggregate type
  Ret += "-n16"; // 16 bit native integer width
  Ret += "-S16"; // 16 bit natural stack alignment
  return Ret;
}

static Reloc::Model getEffectiveRelocModel(Optional<Reloc::Model> RM) {
  if (!RM.hasValue())
    return Reloc::Static;
  return *RM;
}

static CodeModel::Model getEffectiveCodeModel(Optional<CodeModel::Model> CM) {
  if (CM)
    return *CM;
  return CodeModel::Small;
}


LaserTargetMachine::LaserTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
				       StringRef FeatureString, const TargetOptions &Options,
				       Optional<Reloc::Model> RM, 
				       Optional<CodeModel::Model> CodeModel,
				       CodeGenOpt::Level OptLevel, bool JIT
				       )
  : LLVMTargetMachine(T, computeDataLayout(TT), TT,
		      CPU, FeatureString, Options, 
		      getEffectiveRelocModel(RM), 
		      getEffectiveCodeModel(CodeModel), OptLevel),
    TLOF(make_unique<LaserELFTargetObjectFile>()),
    DefaultSubtarget(TT, CPU, FeatureString, isLittle, *this), isLittle(/*isLittle*/ true) {
  initAsmInfo();
}

LaserTargetMachine::~LaserTargetMachine () {}    

//void LaserelTargetMachine::anchor() { }
	  
// LaserelTargetMachine::LaserelTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
//                        StringRef FS, const TargetOptions &Options,
//                        Optional<Reloc::Model> RM, CodeModel::Model CM,
//                        CodeGenOpt::Level OL)
//   : LaserTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL, true /* isLittle */) {}

const LaserSubtarget *
LaserTargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");
  std::string CPU = !CPUAttr.hasAttribute(Attribute::None)
    ? CPUAttr.getValueAsString().str()
    : TargetCPU;
  std::string FS = !FSAttr.hasAttribute(Attribute::None)
    ? FSAttr.getValueAsString().str()
    : TargetFS;

  auto &I = SubtargetMap[CPU + FS];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    I = llvm::make_unique<LaserSubtarget>(TargetTriple, CPU, FS, true /* is Little */, *this);
  }
  return I.get();
}

namespace {
  /// Laser Code Generator Pass Configuration Options.
  class LaserPassConfig : public TargetPassConfig {
  public:
    LaserPassConfig(LaserTargetMachine &TM, PassManagerBase *PM)
      : TargetPassConfig(TM, *PM) {}

    LaserTargetMachine &getLaserTargetMachine() const {
      return getTM<LaserTargetMachine>();
    }
    const LaserSubtarget &getLaserSubtarget() const {
      return *getLaserTargetMachine().getSubtargetImpl();
    }

        bool addInstSelector() override;

  };
} // namespace


TargetPassConfig *LaserTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new LaserPassConfig(*this, &PM);
  }

// Install an instruction selector pass using the ISelDag to gen Laser code.
bool LaserPassConfig::addInstSelector() {
  addPass(createLaserISelDag(getLaserTargetMachine()));
  return false;
  }




