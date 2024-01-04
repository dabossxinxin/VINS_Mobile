
SET(CERES_ROOT_DIR "/Users/liuxianxian/Desktop/SLAM/VINS_Mobile_CMAKE/thirdparty/ceres-solver")
SET(CERES_INCLUDE_DIRS  "${CERES_ROOT_DIR}/include" 
						"${CERES_ROOT_DIR}/internal"
						"${CERES_ROOT_DIR}/config"
						"${CERES_ROOT_DIR}/internal/ceres/miniglog")
SET(CERES_LIBRARY ${CERES_ROOT_DIR}/lib/libceres.a)

ADD_LIBRARY(Ceres::ceres STATIC IMPORTED)
SET_TARGET_PROPERTIES(Ceres::ceres PROPERTIES
	IMPORTED_LINK_INTERFACE_LANGUAGES "C;CXX"
        INTERFACE_INCLUDE_DIRECTORIES "${CERES_INCLUDE_DIRS}"
        IMPORTED_LOCATION "${CERES_LIBRARY}")