//===-- LaserAsmPrinter.cpp - Laser LLVM assembly writer ------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to GAS-format SPARC assembly language.
//
//===----------------------------------------------------------------------===//

#include "InstPrinter/LaserInstPrinter.h"
#include "LaserAsmPrinter.h"
#include "Laser.h"
#include "LaserInstrInfo.h"
#include "LaserMCInstLower.h"
#include "LaserTargetStreamer.h"
#include "MCTargetDesc/LaserBaseInfo.h"
#include "MCTargetDesc/LaserMCExpr.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfoImpls.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "asm-printer"


bool LaserAsmPrinter::runOnMachineFunction(MachineFunction &MF) {
  LaserFI = MF.getInfo<LaserMachineFunctionInfo>();
  AsmPrinter::runOnMachineFunction(MF);
  return true;
}

//- EmitInstruction() must exists or will have run time error.
void LaserAsmPrinter::EmitInstruction(const MachineInstr *MI)
{
  if (MI->isDebugValue()) {
    SmallString<128> Str;
    raw_svector_ostream OS(Str);

    PrintDebugValueComment(MI, OS);
    return;
  }

  //@print out instruction:
  // Print out both ordinary instruction and boudle instruction
  MachineBasicBlock::const_instr_iterator I = MI->getIterator();
  MachineBasicBlock::const_instr_iterator E = MI->getParent()->instr_end();

  do {
    if (I->isPseudo())
      llvm_unreachable("Pseudo opcode found in EmitInstruction()");
    MCInst TmpInst0;
    MCInstLowering.Lower(&*I, TmpInst0);
    OutStreamer->EmitInstruction(TmpInst0, getSubtargetInfo());
  } while ((++I != E) && I->isInsideBundle()); // Delay slot check
}

// Create a bitmask with all callee saved registers for CPU or Floating Point
// registers. For CPU registers consider LR, GP and FP for saving if necessary.
void LaserAsmPrinter::printSavedRegsBitmask(raw_ostream &O) {
  // CPU and FPU Saved Registers Bitmasks
  // unsigned CPUBitmask = 0;
  // int CPUTopSavedRegOff;
  // Set the CPU and FPU Bitmasks

  // const MachineFrameInfo &MFI = MF->getFrameInfo();
  // const TargetRegisterInfo *TRI = MF->getSubtarget().getRegisterInfo();
  // const std::vector<CalleeSavedInfo> &CSI = MFI.getCalleeSavedInfo();
  // size of stack area to which FP callee-saved regs are saved.
  // unsigned CPURegSize = LASER::GNPRegsRegClass.getSize();
  // unsigned i = 0, e = CSI.size();
  // // Set CPU Bitmask.
  // for (; i != e; ++i) {
  //   unsigned Reg = CSI[i].getReg();
  //   unsigned RegNum = TRI->getEncodingValue(Reg);
  //   CPUBitmask |= (1 << RegNum);
  // }
  // CPUTopSavedRegOff = CPUBitmask ? -CPURegSize : 0;
  // // Print CPUBitmask
  // O << "\t.mask \t"; printHex32(CPUBitmask, O);
  // O << ',' << CPUTopSavedRegOff << '\n';
}

// Print a 32 bit hex number with all numbers.
void LaserAsmPrinter::printHex32(unsigned Value, raw_ostream &O) {
  O << "0x";
  for (int i = 7; i >= 0; i--)
    O.write_hex((Value & (0xF << (i*4))) >> (i*4));
}


void LaserAsmPrinter::EmitFunctionEntryLabel() {
  if (OutStreamer->hasRawTextSupport())
    OutStreamer->EmitRawText("\t.ent\t" + Twine(CurrentFnSym->getName()));
  OutStreamer->EmitLabel(CurrentFnSym);
}
// .frame $sp,8,$pc
// .mask 0x00000000,0
//-> .set noreorder
//@-> .set nomacro
/// EmitFunctionBodyStart - Targets can override this to emit stuff before
/// the first basic block in the function.
void LaserAsmPrinter::EmitFunctionBodyStart() {
  MCInstLowering.Initialize(&MF->getContext());
  if (OutStreamer->hasRawTextSupport()) {
    SmallString<128> Str;
    raw_svector_ostream OS(Str);
    printSavedRegsBitmask(OS);
    OutStreamer->EmitRawText(OS.str());
    OutStreamer->EmitRawText(StringRef("\t.set\tnoreorder"));
    OutStreamer->EmitRawText(StringRef("\t.set\tnomacro"));
    if (LaserFI->getEmitNOAT())
      OutStreamer->EmitRawText(StringRef("\t.set\tnoat"));
  }
}

void LaserAsmPrinter::EmitFunctionBodyEnd() {
  // There are instruction for this macros, but they must
  // always be at the function end, and we can't emit and
  // break with BB logic.
  if (OutStreamer->hasRawTextSupport()) {
    if (LaserFI->getEmitNOAT())
      OutStreamer->EmitRawText(StringRef("\t.set\tat"));
    OutStreamer->EmitRawText(StringRef("\t.set\tmacro"));
    OutStreamer->EmitRawText(StringRef("\t.set\treorder"));
    OutStreamer->EmitRawText("\t.end\t" + Twine(CurrentFnSym->getName()));
  }
}

void LaserAsmPrinter::EmitStartOfAsmFile(Module &M) {
  // FIXME: Use SwitchSection.
  // Tell the assembler which ABI we are using
  if (OutStreamer->hasRawTextSupport())
    OutStreamer->EmitRawText("\t.section .mdebug.");
  // return to previous section
  if (OutStreamer->hasRawTextSupport())
    OutStreamer->EmitRawText(StringRef("\t.previous"));
}

void LaserAsmPrinter::PrintDebugValueComment(const MachineInstr *MI,
					     raw_ostream &OS) {
  // TODO: implement
  OS << "PrintDebugValueComment()";
}

// char *LaserAsmPrinter::getRegisterName(unsigned RegNo) {
//   return LaserInstPrinter::getRegisterName(RegNo);
// }


// Force static initialization.
extern "C" void LLVMInitializeLaserAsmPrinter() {
  RegisterAsmPrinter<LaserAsmPrinter> Z(getTheLaserTarget());
  }

