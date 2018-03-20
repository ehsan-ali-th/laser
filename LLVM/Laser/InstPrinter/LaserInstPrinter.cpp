//===-- LaserInstPrinter.cpp - Convert Laser MCInst to assembly syntax -----==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This class prints an Laser MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "LaserInstPrinter.h"
#include "LaserInstrInfo.h"
#include "MCTargetDesc/LaserMCExpr.h"
#include "Laser.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "asm-printer"

#define GET_INSTRUCTION_NAME
#define PRINT_ALIAS_INSTR
#include "LaserGenAsmWriter.inc"

void LaserInstPrinter::printRegName(raw_ostream &OS, unsigned RegNo) const
{
  //- getRegisterName(RegNo) defined in Cpu0GenAsmWriter.inc which indic
  //   Cpu0.td.
  OS << '%' << StringRef(getRegisterName(RegNo)).lower();
}

void LaserInstPrinter::printInst(const MCInst *MI, raw_ostream &O,
				 StringRef Annot, const MCSubtargetInfo &STI) {

  // Try to print any aliases first.
  if (!printAliasInstr(MI, STI, O))
    printInstruction(MI, STI, O);
  
  printAnnotation(O, Annot);
}


void LaserInstPrinter::printOperand(const MCInst *MI, unsigned OpNo, const MCSubtargetInfo &STI, raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);
  if (Op.isReg()) {
    printRegName(O, Op.getReg());
    return;
  }
  if (Op.isImm()) {
    O << Op.getImm();
    return;
  }
  assert(Op.isExpr() && "unknown operand kind in printOperand");
  //printExpr(Op.getExpr(), &MAI, O);
  Op.getExpr()->print(O, &MAI, true);
}

void LaserInstPrinter::printUnsignedImm (const MCInst *MI, unsigned OpNo, const MCSubtargetInfo &STI, raw_ostream &O) {
  const MCOperand &MO = MI->getOperand(OpNo);
  if (MO.isImm())
    O << (unsigned short int)MO.getImm();
  else
    printOperand(MI, OpNo, STI, O);
}

void LaserInstPrinter::printImm(const MCInst *MI, unsigned OpNo, const MCSubtargetInfo &STI, raw_ostream &O) {
  const MCOperand &MO = MI->getOperand(OpNo);
  if (MO.isImm())
    O << "#" << (short int)MO.getImm();
  else
    printOperand(MI, OpNo, STI, O);
}



// Print a 'memsrc' operand which is a [Register].
void LaserInstPrinter::printAddrModeMemSrc(const MCInst *MI, unsigned OpNo, const MCSubtargetInfo &STI, raw_ostream &O) {
  O << "[";
  printOperand(MI, OpNo, STI, O);
  O << "]";  
}

// Print a 'LASERimm11op' operand which is a [#BB].
// void LaserInstPrinter::printJumpTarget(const MCInst *MI, unsigned OpNo, const MCSubtargetInfo &STI, raw_ostream &O) {
//   O << "[";
//   printOperand(MI, OpNo, STI, O);
//   O << "]";  
// }


