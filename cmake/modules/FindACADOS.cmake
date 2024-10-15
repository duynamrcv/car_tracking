#  Try to find Gflags lib
#  The following are set after configuration is done:
#  ACADOS_FOUND
#  ACADOS_INCLUDE_DIR
#  ACADOS_LIBRARY_DIR
#  ACADOS_LIBRARIES

set(ACADOS_INCLUDE_DIR "")
set(ACADOS_LIBRARY_DIR "")
set(ACADOS_LIBRARIES "")

# Find package
set(ACADOS_PATH ${VENDOR_PREFIX}/acados)
find_file(ACADOS_FOUND "acados/utils/math.h" PATHS ${ACADOS_PATH}/include/${ARCH})
if (ACADOS_FOUND)
    list(APPEND ACADOS_INCLUDE_DIR
        ${ACADOS_PATH}/include/${ARCH}
        ${ACADOS_PATH}/include/${ARCH}/blasfeo/include
        ${ACADOS_PATH}/include/${ARCH}/hpipm/include
        ${ACADOS_PATH}/include/${ARCH}/acados
    )
    list(APPEND ACADOS_LIBRARY_DIR
        ${ACADOS_PATH}/lib/${ARCH}/
    )
    list(APPEND ACADOS_LIBRARIES
        acados
        blasfeo
        hpipm
        qpOASES_e
    )
else (ACADOS_FOUND)
    message(FATAL_ERROR "=====\ACADOS not found! Please install ACADOS first.\n")
endif(ACADOS_FOUND)


