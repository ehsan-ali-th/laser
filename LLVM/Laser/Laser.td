//===-- Laser.td - Describe the Laser Target Machine -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces which we are implementing
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// LASER Subtarget features.
//


//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "LaserRegisterInfo.td"
include "LaserCallingConv.td"
include "LaserSchedule.td"
include "LaserInstrInfo.td"

def LaserInstrInfo : InstrInfo;

def LaserAsmParser : AsmParser { bit ShouldEmitMatchRegisterName = 0; }

def LaserAsmParserVariant : AsmParserVariant {
    int Variant = 0;
    // Recognize hard coded registers.
    string RegisterPrefix = "%";
}      

//===----------------------------------------------------------------------===//
// LASER processors supported.
//===----------------------------------------------------------------------===//

class Proc<string Name, list<SubtargetFeature> Features>
    : Processor<Name, NoItineraries, Features>;

def : Proc<"generic", []>;

//===----------------------------------------------------------------------===//
// Declare the target which we are implementing
//===----------------------------------------------------------------------===//

def LaserAsmWriter : AsmWriter {
  string AsmWriterClassName = "InstPrinter";
  int PassSubtarget = 1;
  int Variant = 0;
}

def Laser : Target {
  // Pull in Instruction Info:
  let InstructionSet = LaserInstrInfo;
  let AssemblyParsers = [ LaserAsmParser ];
  let AssemblyParserVariants = [LaserAsmParserVariant];
  let AssemblyWriters = [ LaserAsmWriter ];
}
