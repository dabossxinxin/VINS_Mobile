
include_directories(SYSTEM ${THIRD_PARTY_PATH}/OpenCV ${THIRD_PARTY_PATH}/OpenCV/opencv2.framework/Headers)
find_library(OpenCV_LIBRARY NAMES opencv2 HINTS ${THIRD_PARTY_PATH}/OpenCV/opencv2.framework)