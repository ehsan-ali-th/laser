//===-- LaserMCExpr.cpp - Laser specific MC expression classes --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation of the assembly expression modifiers
// accepted by the Laser architecture (e.g. "%hi", "%lo", ...).
//
//===----------------------------------------------------------------------===//

#include "LaserMCExpr.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCObjectStreamer.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Object/ELF.h"

using namespace llvm;

#define DEBUG_TYPE "lasermcexpr"

const LaserMCExpr *LaserMCExpr::create(VariantKind Kind,
                                     const MCExpr *Expr, MCContext &Ctx) {
  return new (Ctx) LaserMCExpr(Kind, Expr);
}

void LaserMCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  if (Kind == VK_LASER_NONE) {
    Expr->print(OS, MAI);
    return;
  }

  switch (Kind) {
    //case CEK_None:
  default:
    llvm_unreachable("CEK_None and CEK_Special are invalid");
    break;
  case VK_LASER_CALL16:
    OS << "@call16";
    break;
  case VK_LASER_PC11:
    // OS << "@jmp11";
    break;
  case VK_LASER_GV16:
    OS << "@gv16";
    break;
  }


  if (Kind == VK_LASER_PC11) {
    const MCExpr *Expr = getSubExpr();
    OS << '@';
    Expr->print(OS, MAI);
  }
  else {
    OS << '[';
    const MCExpr *Expr = getSubExpr();
    Expr->print(OS, MAI);
    OS << ']';
  }
}


bool
LaserMCExpr::evaluateAsRelocatableImpl(MCValue &Res,
                                       const MCAsmLayout *Layout,
                                       const MCFixup *Fixup) const {
  return getSubExpr()->evaluateAsRelocatable(Res, Layout, Fixup);
}


void LaserMCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}
