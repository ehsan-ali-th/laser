//===-- LaserAsmBackend.cpp - Laser Asm Backend  ----------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the LaserAsmBackend class.
//
//===----------------------------------------------------------------------===//
//

#include "MCTargetDesc/LaserFixupKinds.h"
#include "MCTargetDesc/LaserAsmBackend.h"

#include "MCTargetDesc/LaserMCTargetDesc.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

//@adjustFixupValue {
// Prepare value for the target space for it
// static unsigned adjustFixupValue(const MCFixup &Fixup, uint64_t Value,
//                                  MCContext *Ctx = nullptr) {

//   unsigned Kind = Fixup.getKind();

//   // Add/subtract and shift
//   switch (Kind) {
//   default:
//     llvm_unreachable("Unknown fixup kind!");
//   case Laser::fixup_Laser_PC16:
//   case Laser::fixup_Laser_PC11:
//     Value = 2;
//     break;
//   case Laser::fixup_Laser_HI16:
//     // Get the higher 16-bits. Also add 1 if bit 15 is 1.
//     Value = ((Value + 0x8000) >> 16) & 0xffff;
//     break;
//   case Laser::fixup_Laser_LO16:
//     break;
//   case Laser::fixup_Laser_CALL16:
//     Value = 2;
//     break;
//   }
//     return Value;
// }
// //@adjustFixupValue }

std::unique_ptr<MCObjectWriter>
LaserAsmBackend::createObjectWriter(raw_pwrite_stream &OS) const {
  return createLaserELFObjectWriter(OS,
    MCELFObjectTargetWriter::getOSABI(OSType));
}

/// ApplyFixup - Apply the \p Value for given \p Fixup into the provided
/// data fragment, at the offset specified by the fixup and following the
/// fixup kind as appropriate.
void LaserAsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                                 const MCValue &Target,
                                 MutableArrayRef<char> Data, uint64_t Value,
                                 bool IsResolved) const {
  MCFixupKind Kind = Fixup.getKind();
  //Value = adjustFixupValue(Fixup, Value);

  if (!Value)
    return; // Doesn't change encoding.


  // Where do we start in the object
  unsigned Offset = Fixup.getOffset();
  // Number of bytes we need to fixup
  unsigned NumBytes = (getFixupKindInfo(Kind).TargetSize + 7) / 8;
  // Used to point to big endian bytes
  unsigned FullSize;

  switch ((unsigned)Kind) {
  case Laser::fixup_Laser_CALL16:
    FullSize = 4;
    break;
  case Laser::fixup_Laser_GV16:
    FullSize = 4;
    break;
  default:
    FullSize = 2;
    break;
  }

  // Grab current value, if any, from bits.
  uint64_t CurVal = 0;

  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned Idx = (FullSize - 1 - i);
    CurVal |= (uint64_t)((uint8_t)Data[Offset + Idx]) << (i*8);
  }

  uint64_t Mask = ((uint64_t)(-1) >>
                    (64 - getFixupKindInfo(Kind).TargetSize));
  CurVal |= Value & Mask;

  // Write out the fixed up bytes back to the code/data bits.
  for (unsigned i = 0; i != NumBytes; ++i) {
    unsigned Idx = (FullSize - 1 - i);
    Data[Offset + Idx] = (uint8_t)((CurVal >> (i*8)) & 0xff);
  }
}

//@getFixupKindInfo {
const MCFixupKindInfo &LaserAsmBackend::
getFixupKindInfo(MCFixupKind Kind) const {
  const static MCFixupKindInfo Infos[Laser::NumTargetFixupKinds] = {
    // This table *must* be in same the order of fixup_* kinds in
    // LaserFixupKinds.h.
    //
    // name                        offset  bits  flags
    { "fixup_Laser_CALL16",        0,     16,   0 },
    { "fixup_Laser_PC11",          0,     11,   0 },
    { "fixup_Laser_GV16",          0,     16,   0 },
  };

  if (Kind < FirstTargetFixupKind)
    return MCAsmBackend::getFixupKindInfo(Kind);

  assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
         "Invalid kind!");
  return Infos[Kind - FirstTargetFixupKind];
}
//@getFixupKindInfo }

/// WriteNopData - Write an (optimal) nop sequence of Count bytes
/// to the given output. If the target cannot generate such a sequence,
/// it should return an error.
///
/// \return - True on success.
bool LaserAsmBackend::writeNopData(uint64_t Count, MCObjectWriter *OW) const {
  if ((Count % 2) != 0)
    return false;

  for (uint64_t i = 0; i < Count; i += 2)
    OW->write16(0xF800); // 0xF800 = NOP instruction

  return true;
}

// MCAsmBackend
MCAsmBackend *llvm::createLaserAsmBackend16(const Target &T,
                                          const MCSubtargetInfo &STI,
                                          const MCRegisterInfo & /*MRI*/,
                                          const MCTargetOptions & /*Options*/) {
    const Triple &TT = STI.getTargetTriple();
  if (!TT.isOSBinFormatELF())
    llvm_unreachable("OS not supported");

  return new LaserAsmBackend(T, TT.getOS());
}

