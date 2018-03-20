//===-- LaserMCTargetDesc.cpp - Laser Target Descriptions -----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Laser specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "LaserMCTargetDesc.h"

#include "InstPrinter/LaserInstPrinter.h"
#include "LaserMCAsmInfo.h"
#include "LaserTargetStreamer.h"

#include "llvm/MC/MachineLocation.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrAnalysis.h"
#include "llvm/MC/MCInstPrinter.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#include "LaserGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "LaserGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "LaserGenRegisterInfo.inc"

/// Select the Laser Architecture Feature for the given triple and cpu name.
/// The function will be called at command 'llvm-objdump -d' for Laser elf input.
static StringRef selectLaserArchFeature(const Triple &TT, StringRef CPU) {
  std::string LaserArchFeature;
  if (CPU.empty() || CPU == "generic") {
    if (TT.getArch() == Triple::laser) {
	LaserArchFeature = "+laserarch";
    }
  }
  return LaserArchFeature;
}

static MCInstrInfo *createLaserMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitLaserMCInstrInfo(X); // defined in LaserGenInstrInfo.inc
  return X;
}

static MCRegisterInfo *createLaserMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitLaserMCRegisterInfo(X, LASER::R8); // defined in LaserGenRegisterInfo.inc
  return X;
}

static MCSubtargetInfo *createLaserMCSubtargetInfo(const Triple &TT,
						  StringRef CPU, StringRef FS) {
  std::string ArchFS = selectLaserArchFeature(TT,CPU);
  if (!FS.empty()) {
    if (!ArchFS.empty())
      ArchFS = ArchFS + "," + FS.str();
    else
      ArchFS = FS;
  }
  return createLaserMCSubtargetInfoImpl(TT, CPU, ArchFS);
  // createLaserMCSubtargetInfoImpl defined in LaserGenSubtargetInfo.inc
}

static MCAsmInfo *createLaserMCAsmInfo(const MCRegisterInfo &MRI,
				      const Triple &TT) {
  MCAsmInfo *MAI = new LaserELFMCAsmInfo(TT);
  unsigned SP = MRI.getDwarfRegNum(LASER::SP, true);
  MCCFIInstruction Inst = MCCFIInstruction::createDefCfa(nullptr, SP, 0);
  MAI->addInitialFrameState(Inst);
  return MAI;
}

static MCInstPrinter *createLaserMCInstPrinter(const Triple &T,
					      unsigned SyntaxVariant,
					      const MCAsmInfo &MAI,
					      const MCInstrInfo &MII,
					      const MCRegisterInfo &MRI) {
  return new LaserInstPrinter(MAI, MII, MRI);
}

static MCStreamer *createMCStreamer(const Triple &T, MCContext &Context,
                                    std::unique_ptr<MCAsmBackend> &&MAB,
                                    raw_pwrite_stream &OS,
                                    std::unique_ptr<MCCodeEmitter> &&Emitter,
                                    bool RelaxAll) {
  if (!T.isOSBinFormatELF())
    llvm_unreachable("OS not supported");

  return createELFStreamer(Context, std::move(MAB), OS, std::move(Emitter),
                           RelaxAll);

}
static MCTargetStreamer *createLaserAsmTargetStreamer(MCStreamer &S,
						     formatted_raw_ostream &OS,
						     MCInstPrinter *InstPrint,
						     bool isVerboseAsm) {
  return new LaserTargetAsmStreamer(S, OS);
}

extern "C" void LLVMInitializeLaserTargetMC() {
  // for (Target *T : {&TheLaserTarget}) {
    // Register the elf streamer.
    TargetRegistry::RegisterELFStreamer(getTheLaserTarget(), 
					createMCStreamer);

    // Register the asm target streamer.
    TargetRegistry::RegisterAsmTargetStreamer(getTheLaserTarget(), 
					      createLaserAsmTargetStreamer);

    // Register the MC Code Emitter
    TargetRegistry::RegisterMCCodeEmitter(getTheLaserTarget(),
					  createLaserMCCodeEmitter);
    // Register the asm backend.
    TargetRegistry::RegisterMCAsmBackend(getTheLaserTarget(),
					 createLaserAsmBackend16);

    // Register the MC asm info.
    RegisterMCAsmInfoFn X(getTheLaserTarget(), createLaserMCAsmInfo);

    // Register the MC instruction info.
    TargetRegistry::RegisterMCInstrInfo(getTheLaserTarget(), createLaserMCInstrInfo);

    // Register the MC register info.
    TargetRegistry::RegisterMCRegInfo(getTheLaserTarget(), createLaserMCRegisterInfo);

    // Register the MC subtarget info.
    TargetRegistry::RegisterMCSubtargetInfo(getTheLaserTarget(),
					    createLaserMCSubtargetInfo);
    // Register the MCInstPrinter.
    TargetRegistry::RegisterMCInstPrinter(getTheLaserTarget(),
    					  createLaserMCInstPrinter);
}

