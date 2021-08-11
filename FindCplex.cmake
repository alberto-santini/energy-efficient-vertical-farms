set(_supported_components ConstraintProgramming)

find_path(Cplex_INCLUDE_DIR
        NAMES ilcplex/ilocplex.h
        HINTS ${CPLEX_DIR} $ENV{CPLEX_HOME}
        PATH_SUFFIXES cplex/include include)

find_path(Cplex_Concert_INCLUDE_DIR
        NAMES ilconcert/iloenv.h
        HINTS ${CPLEX_DIR} $ENV{CPLEX_HOME}
        PATH_SUFFIXES concert/include include)

find_path(Cplex_ConstraintProgramming_INCLUDE_DIR
        NAMES ilcp/cp.h
        HINTS ${CPLEX_DIR} $ENV{CPLEX_HOME}
        PATH_SUFFIXES cpoptimizer/include include)

find_library(Cplex_LIBRARY
        NAMES cplex ilocplex
        HINTS ${CPLEX_DIR} $ENV{CPLEX_HOME}
        PATH_SUFFIXES cplex/lib/x86-64_linux/static_pic cplex/lib/x86-64_osx/static_pic lib)

find_library(Cplex_Concert_LIBRARY
        NAMES concert
        HINTS ${CPLEX_DIR} $ENV{CPLEX_HOME}
        PATH_SUFFIXES concert/lib/x86-64_linux/static_pic concert/lib/x86-64_osx/static_pic lib)

find_library(Cplex_ConstraintProgramming_LIBRARY
        NAMES cp
        HINTS ${CPLEX_DIR} $ENV{CPLEX_HOME}
        PATH_SUFFIXES cpoptimizer/lib/x86-64_linux/static_pic cpoptimizer/lib/x86-64_osx/static_pic lib)

if(Cplex_ConstraintProgramming_INCLUDE_DIR AND Cplex_ConstraintProgramming_LIBRARY)
    set(Cplex_ConstraintProgramming_FOUND TRUE)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Cplex
        REQUIRED_VARS Cplex_INCLUDE_DIR Cplex_Concert_INCLUDE_DIR Cplex_LIBRARY Cplex_Concert_LIBRARY)

if(Cplex_FOUND AND NOT TARGET Cplex::Cplex)
    set(Cplex_INCLUDE_DIRS ${Cplex_INCLUDE_DIR} ${Cplex_Concert_INCLUDE_DIR})
    set(Cplex_LIBRARIES ${Cplex_Concert_LIBRARY} ${Cplex_LIBRARY})
    add_library(Cplex::Cplex UNKNOWN IMPORTED)
    set_target_properties(Cplex::Cplex PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${Cplex_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${Cplex_LIBRARIES}")
endif()

mark_as_advanced(Cplex_INCLUDE_DIR Cplex_LIBRARY Cplex_Concert_INCLUDE_DIR Cplex_Concert_LIBRARY)

if(Cplex_ConstraintProgramming_FOUND AND NOT TARGET Cplex::ConstraintProgramming)
    set(Cplex_ConstraintProgramming_INCLUDE_DIRS ${Cplex_ConstraintProgramming_INCLUDE_DIR})
    set(Cplex_ConstraintProgramming_LIBRARIES ${Cplex_ConstraintProgramming_LIBRARY})
    add_library(Cplex::ConstraintProgramming UNKNOWN IMPORTED)
    set_target_properties(Cplex::ConstraintProgramming PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${Cplex_ConstraintProgramming_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${Cplex_ConstraintProgramming_LIBRARIES}")
endif()

mark_as_advanced(Cplex_ConstraintProgramming_INCLUDE_DIR Cplex_ConstraintProgramming_LIBRARY)