
########################################################
#													   						  #
#			          				编译DBoW库			           					  #
#													   						  #
########################################################
file(GLOB DBoWHdrs		"./DBoW/*.h")
file(GLOB DUtilsHdrs	"./DUtils/*.h")
file(GLOB DVsionHdrs	"./DVision/*.h")

file(GLOB DBoWSrcs		"./DBoW/*.cpp")
file(GLOB DUtilsSrcs	"./DUtils/*.cpp")
file(GLOB DVsionSrcs	"./DVision/*.cpp")

source_group("DBoW"		FILES ${DBoWHdrs} ${DBoWSrcs})
source_group("DUtils" 	FILES ${DUtilsHdrs} ${DUtilsSrcs})
source_group("DVsion" 	FILES ${DVsionHdrs} ${DVsionSrcs})

add_library(DBoW 
	${DBoWHdrs}
	${DBoWSrcs}
	${DUtilsHdrs}
	${DUtilsSrcs}
	${DVsionHdrs}
	${DVsionSrcs}
)

target_link_libraries(DBoW Boost::container ${OpenCV_LIBRARY})
