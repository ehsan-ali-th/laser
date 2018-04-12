//===-- LaserMCCodeEmitter.cpp - Convert Laser Code to Machine Code ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the LaserMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//
//

#include "LaserMCCodeEmitter.h"

#include "MCTargetDesc/LaserBaseInfo.h"
#include "MCTargetDesc/LaserFixupKinds.h"
#include "MCTargetDesc/LaserMCExpr.h"
#include "MCTargetDesc/LaserMCTargetDesc.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "mccodeemitter"

#define GET_INSTRMAP_INFO
#include "LaserGenInstrInfo.inc"
#undef GET_INSTRMAP_INFO

namespace llvm {
MCCodeEmitter *createLaserMCCodeEmitter(const MCInstrInfo &MCII,
                                               const MCRegisterInfo &MRI,
                                               MCContext &Ctx) {
  return new LaserMCCodeEmitter(MCII, Ctx);
}

} // End of namespace llvm

void LaserMCCodeEmitter::EmitByte(unsigned char C, raw_ostream &OS) const {
  OS << (char)C;
}

void LaserMCCodeEmitter::EmitInstruction(uint64_t Val, unsigned Size, raw_ostream &OS) const {
  // Output the instruction encoding in big endian byte order.
  for (unsigned i = 0; i < Size; ++i) {
    unsigned Shift = (Size - 1 - i) * 8;
    EmitByte((Val >> Shift) & 0xff, OS);
  }
}

/// encodeInstruction - Emit the instruction.
/// Size the instruction (currently only 2 bytes)
void LaserMCCodeEmitter::
encodeInstruction(const MCInst &MI, raw_ostream &OS,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const
{
  unsigned Binary = getBinaryCodeForInstr(MI, Fixups, STI);

  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  uint64_t TSFlags = Desc.TSFlags;

  // Pseudo instructions don't get encoded and shouldn't be here
  // in the first place!
  if ((TSFlags & LaserII::FormMask) == LaserII::LaserPseudo)
    llvm_unreachable("Pseudo opcode found in encodeInstruction()");

  // For now all instructions are 2 bytes except IMD which is 4 byte
  // We set the instruction Size in LaserInstFormat.td
  int Size = Desc.getSize(); 

  EmitInstruction(Binary, Size, OS);
  
  // Output the bits in big-endian byte order.
  //support::endian::Writer<support::big>(OS).write<uint16_t>(Binary);

}

unsigned LaserMCCodeEmitter::
getExprOpValue(const MCExpr *Expr,SmallVectorImpl<MCFixup> &Fixups,
               const MCSubtargetInfo &STI) const {
  MCExpr::ExprKind Kind = Expr->getKind();

  if (Kind == MCExpr::Constant) {
    return cast<MCConstantExpr>(Expr)->getValue();
  }

  if (Kind == MCExpr::Binary) {
    unsigned Res = getExprOpValue(cast<MCBinaryExpr>(Expr)->getLHS(), Fixups, STI);
    Res += getExprOpValue(cast<MCBinaryExpr>(Expr)->getRHS(), Fixups, STI);
    return Res;
  }

  assert(isa<LaserMCExpr>(Expr) || Expr->getKind() == MCExpr::SymbolRef);

  // Target specific expression. 
  if (Kind == MCExpr::Target) {
    if(const LaserMCExpr *LaserExpr = dyn_cast<LaserMCExpr>(Expr)) {

      Laser::Fixups FixupKind = Laser::Fixups(0);
      switch (LaserExpr->getKind()) {
      default: llvm_unreachable("Unsupported fixup kind for target expression!");
	//@switch {
	//    switch(cast<MCSymbolRefExpr>(Expr)->getKind()) {
	//@switch }
      case LaserMCExpr::VK_LASER_NONE:
	break;
      case LaserMCExpr::VK_LASER_CALL16:
	FixupKind = Laser::fixup_Laser_CALL16;
	break;
      case LaserMCExpr::VK_LASER_GV16:
	FixupKind = Laser::fixup_Laser_GV16;
	break;
      } // switch
      Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind(FixupKind)));
      return 0;
    }

  }



  // All of the information is in the fixup.
  return 0;
}

/// getMachineOpValue - Return binary encoding of operand. If the machine
/// operand requires relocation, record the relocation and return zero.
unsigned LaserMCCodeEmitter::
getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  if (MO.isReg()) {
    unsigned Reg = MO.getReg();
    unsigned RegNo = Ctx.getRegisterInfo()->getEncodingValue(Reg);
    return RegNo;
  } else if (MO.isImm()) {
    return static_cast<unsigned>(MO.getImm());
  } 

  // MO must be an Expr.
  assert(MO.isExpr());
  return getExprOpValue(MO.getExpr(),Fixups, STI);
}

/// getMemEncoding - Return binary encoding of memory related operand.
/// If the offset operand requires relocation, record the relocation.
unsigned
LaserMCCodeEmitter::getMemEncoding(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups,
                                  const MCSubtargetInfo &STI) const {
  // Base register is encoded in bits 20-16, offset is encoded in bits 15-0.
  assert(MI.getOperand(OpNo).isReg());
  unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI) << 16;
  unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  return (OffBits & 0xFFFF) | RegBits;
}

/// getMemSrcValue - Return binary encoding of memory related operand.
/// If the offset operand requires relocation, record the relocation.
unsigned
LaserMCCodeEmitter::getMemSrcValue(const MCInst &MI, unsigned OpNo,
                                  SmallVectorImpl<MCFixup> &Fixups,
                                  const MCSubtargetInfo &STI) const {
  unsigned Bits = 0;
  const MCOperand &RegMO = MI.getOperand(OpNo);
  const MCOperand &ImmMO = MI.getOperand(OpNo + 1);
  assert(ImmMO.getImm() >= 0);
  Bits |= (getMachineOpValue(MI, RegMO, Fixups, STI) << 12);
  Bits |= (unsigned)ImmMO.getImm() & 0xfff;
  return Bits;

  // Base register is encoded in bits 20-16, offset is encoded in bits 15-0.
  // assert(MI.getOperand(OpNo).isReg());
  // unsigned RegBits = getMachineOpValue(MI, MI.getOperand(OpNo), Fixups, STI) << 16;
  // unsigned OffBits = getMachineOpValue(MI, MI.getOperand(OpNo+1), Fixups, STI);

  // return (OffBits & 0xFFFF) | RegBits;
}

unsigned LaserMCCodeEmitter::getJumpTarget11OpValue(const MCInst &MI, unsigned OpNo,
						   SmallVectorImpl<MCFixup> &Fixups,
						   const MCSubtargetInfo &STI) const {
  unsigned Opcode = MI.getOpcode();
  const MCOperand &MO = MI.getOperand(OpNo);
  // If the destination is an immediate, we have nothing to do.
  if (MO.isImm()) return MO.getImm();
  assert(MO.isExpr() && "getJumpTargetOpValue expects only expressions");

  const MCExpr *Expr = MO.getExpr();
  
  Laser::Fixups FixupKind = Laser::fixup_Laser_PC11;

  if (Opcode == LASER::JMP)
    Fixups.push_back(MCFixup::create(0, Expr,
				     MCFixupKind(FixupKind)));
  else
    llvm_unreachable("unexpect opcode in getJumpAbsoluteTargetOpValue()");

  return 0;
}


#include "LaserGenMCCodeEmitter.inc"

