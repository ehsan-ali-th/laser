//===-- LaserAsmPrinter.h - Laser LLVM Assembly Printer ----------*- C++ -*--===//
//
// The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Laser Assembly printer class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_LASER_LASERASMPRINTER_H
#define LLVM_LIB_TARGET_LASER_LASERASMPRINTER_H

#include "LaserMachineFunctionInfo.h"
#include "LaserMCInstLower.h"
#include "LaserSubtarget.h"
#include "LaserTargetMachine.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

  class MCStreamer;
  class MachineInstr;
  class MachineBasicBlock;
  class Module;
  class raw_ostream; 

  class LLVM_LIBRARY_VISIBILITY LaserAsmPrinter : public AsmPrinter {

    void EmitInstrWithMacroNoAT(const MachineInstr *MI);

  private:
    // lowerOperand - Convert a MachineOperand into the equivalent MCOperand.
    bool lowerOperand(const MachineOperand &MO, MCOperand &MCOp);

  public:
    const LaserSubtarget *Subtarget;
    const LaserMachineFunctionInfo *LaserFI;
    LaserMCInstLower MCInstLowering;

    explicit LaserAsmPrinter(TargetMachine &TM,
			     std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)),
      MCInstLowering(*this) {
      Subtarget = static_cast<LaserTargetMachine &>(TM).getSubtargetImpl();
    }

    virtual StringRef getPassName() const override {
      return "Laser Assembly Printer";
    }

    virtual bool runOnMachineFunction(MachineFunction &MF) override;

    //- EmitInstruction() must exists or will have run time error.
    void EmitInstruction(const MachineInstr *MI) override;

    // static const char *getRegisterName(unsigned RegNo); 

    void printSavedRegsBitmask(raw_ostream &O);
    void printHex32(unsigned Value, raw_ostream &O);
    // void emitFrameDirective();
    const char *getCurrentABIString() const;
    void EmitFunctionEntryLabel() override;
    void EmitFunctionBodyStart() override;
    void EmitFunctionBodyEnd() override;
    void EmitStartOfAsmFile(Module &M) override;
    void PrintDebugValueComment(const MachineInstr *MI, raw_ostream &OS);
  };
}
#endif
