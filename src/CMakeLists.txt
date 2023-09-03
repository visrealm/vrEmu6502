

add_library(vrEmu6502 vrEmu6502.c)

if (WIN32)
  if (BUILD_SHARED_LIBS)
     add_definitions(-DVR_6502_EMU_COMPILING_DLL)
   else()
     add_definitions(-DVR_6502_EMU_STATIC)
   endif()
endif()

target_include_directories (vrEmu6502 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
