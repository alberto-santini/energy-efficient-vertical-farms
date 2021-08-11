find_path(Gurobi_INCLUDE_DIR
        NAMES gurobi_c++.h
        HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
        PATH_SUFFIXES include)

find_library(Gurobi_LIBRARY
        NAMES gurobi gurobi91 gurobi90
        HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib)

find_library(Gurobi_CXX_LIBRARY
        NAMES gurobi_c++
        HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Gurobi
        REQUIRED_VARS Gurobi_INCLUDE_DIR Gurobi_LIBRARY Gurobi_CXX_LIBRARY)

if(Gurobi_FOUND AND NOT TARGET Gurobi::Gurobi)
    set(Gurobi_INCLUDE_DIRS ${Gurobi_INCLUDE_DIR})
    set(Gurobi_LIBRARIES ${Gurobi_LIBRARY} ${Gurobi_CXX_LIBRARY})
    add_library(Gurobi::Gurobi UNKNOWN IMPORTED)
    set_target_properties(Gurobi::Gurobi PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${Gurobi_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${Gurobi_LIBRARIES}")
endif()

mark_as_advanced(Gurobi_INCLUDE_DIR Gurobi_LIBRARY Gurobi_CXX_LIBRARY)