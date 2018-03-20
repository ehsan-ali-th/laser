//===-- LaserMCTargetDesc.h - Laser Target Descriptions ---------*- C++ -*-===//
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

#ifndef LLVM_LIB_TARGET_LASER_MCTARGETDESC_LASERMCTARGETDESC_H
#define LLVM_LIB_TARGET_LASER_MCTARGETDESC_LASERMCTARGETDESC_H

#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCTargetOptions.h"
#include "llvm/Support/DataTypes.h"

namespace llvm {
class Target;
class Triple;
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class StringRef;

class raw_ostream;
class raw_pwrite_stream;

  //extern Target TheLaserTarget;

Target &getTheLaserTarget();

MCCodeEmitter *createLaserMCCodeEmitter(const MCInstrInfo &MCII,
                                        const MCRegisterInfo &MRI,
                                        MCContext &Ctx);

MCAsmBackend *createLaserAsmBackend16(const Target &T,
                                          const MCSubtargetInfo &STI,
                                          const MCRegisterInfo & /*MRI*/,
                                          const MCTargetOptions & /*Options*/);

std::unique_ptr<MCObjectWriter> 
createLaserELFObjectWriter(raw_pwrite_stream &OS, uint8_t OSABI);

} // End llvm namespace

// Defines symbolic names for Laser registers.  This defines a mapping from
// register name to register number.
//
#define GET_REGINFO_ENUM
#include "LaserGenRegisterInfo.inc"

// Defines symbolic names for the Laser instructions.
//
#define GET_INSTRINFO_ENUM
#include "LaserGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "LaserGenSubtargetInfo.inc"

#endif
