//===-- LaserTargetStreamer.cpp - Laser Target Streamer Methods -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides Laser specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "InstPrinter/LaserInstPrinter.h"
#include "LaserMCTargetDesc.h"
#include "LaserTargetObjectFile.h"
#include "LaserTargetStreamer.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

LaserTargetStreamer::LaserTargetStreamer(MCStreamer &S)
    : MCTargetStreamer(S) {
}

LaserTargetAsmStreamer::LaserTargetAsmStreamer(MCStreamer &S,
                                             formatted_raw_ostream &OS)
    : LaserTargetStreamer(S), OS(OS) {}

