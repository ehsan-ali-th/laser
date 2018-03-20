//===-- LaserMCInstLower.cpp - Convert Laser MachineInstr to MCInst -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower Laser MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "LaserMCInstLower.h"
#include "LaserAsmPrinter.h"

#include "MCTargetDesc/LaserBaseInfo.h"
#include "Laser.h"
#include "MCTargetDesc/LaserMCExpr.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"

using namespace llvm;

LaserMCInstLower::LaserMCInstLower(LaserAsmPrinter &asmprinter)
  : AsmPrinter(asmprinter) {}

void LaserMCInstLower::Initialize(MCContext* C) {
  Ctx = C;
}

MCOperand LaserMCInstLower::LowerSymbolOperand(const MachineOperand &MO,
                                              MachineOperandType MOTy,
                                              unsigned Offset) const {
  LaserMCExpr::VariantKind Kind =
    (LaserMCExpr::VariantKind)MO.getTargetFlags();
  const MCSymbol *Symbol  = nullptr;

  switch(MO.getTargetFlags()) {
  default: llvm_unreachable("Invalid target flag!");
  case LaserII::MO_NO_FLAG:
    Kind = LaserMCExpr::VK_LASER_NONE;
    break;
  case LaserII::MO_CALL_FLAG:
    Kind = LaserMCExpr::VK_LASER_CALL16;
    break;
  case LaserII::MO_ABS:
    Kind = LaserMCExpr::VK_LASER_GV16;
    break;
  }

  switch (MOTy) {
  default: llvm_unreachable("<unknown operand type>");
  case MachineOperand::MO_GlobalAddress:
    Symbol = AsmPrinter.getSymbol(MO.getGlobal());
    break;
  
  case MachineOperand::MO_MachineBasicBlock:
    Symbol = MO.getMBB()->getSymbol();
    break;

  case MachineOperand::MO_BlockAddress:
    Symbol = AsmPrinter.GetBlockAddressSymbol(MO.getBlockAddress());
    break;

  case MachineOperand::MO_ExternalSymbol:
    Symbol = AsmPrinter.GetExternalSymbolSymbol(MO.getSymbolName());
    break;
  }

  const MCSymbolRefExpr *MCSym = MCSymbolRefExpr::create(Symbol,
                                                         AsmPrinter.OutContext);
  const LaserMCExpr *expr = LaserMCExpr::create(Kind, MCSym,
                                                AsmPrinter.OutContext);
  return MCOperand::createExpr(expr);
}

MCOperand LaserMCInstLower::LowerOperand(const MachineOperand& MO,
					 unsigned offset) const {
  MachineOperandType MOTy = MO.getType();
  switch (MOTy) {
  default: llvm_unreachable("unknown operand type"); break;

  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit()) 
      break;
    return MCOperand::createReg(MO.getReg());

  case MachineOperand::MO_Immediate:
    return MCOperand::createImm(MO.getImm() + offset);


  case MachineOperand::MO_MachineBasicBlock:
  case MachineOperand::MO_GlobalAddress:
  case MachineOperand::MO_BlockAddress:
  case MachineOperand::MO_ExternalSymbol:
  case MachineOperand::MO_JumpTableIndex:
    return LowerSymbolOperand(MO, MOTy, offset);

  case MachineOperand::MO_RegisterMask:
    break;
  }
  return MCOperand();
}

void LaserMCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(MI->getOpcode());
  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    MCOperand MCOp = LowerOperand(MO);
    if (MCOp.isValid())
      OutMI.addOperand(MCOp);
  }
}


