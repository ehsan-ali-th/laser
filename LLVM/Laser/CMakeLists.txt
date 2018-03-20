set(LLVM_TARGET_DEFINITIONS Laser.td)

tablegen(LLVM LaserGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM LaserGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM LaserGenCodeEmitter.inc -gen-emitter)
tablegen(LLVM LaserGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM LaserGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM LaserGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM LaserGenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM LaserGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM LaserGenSubtargetInfo.inc -gen-subtarget)
tablegen(LLVM LaserGenCallingConv.inc -gen-callingconv)

add_public_tablegen_target(LaserCommonTableGen)

add_llvm_target(LaserCodeGen
  LaserAsmPrinter.cpp
  LaserInstrInfo.cpp
  LaserISelDAGToDAG.cpp
  LaserISelLowering.cpp
  LaserFrameLowering.cpp
  LaserMachineFunctionInfo.cpp
  LaserRegisterInfo.cpp
  LaserSubtarget.cpp
  LaserTargetMachine.cpp
  LaserMCInstLower.cpp
  LaserTargetObjectFile.cpp
  )

add_subdirectory(TargetInfo)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(InstPrinter)
add_subdirectory(AsmParser)

