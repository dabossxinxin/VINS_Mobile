
########################################################
#													   						  #
#							编译visual inertial odometry库			   				  #
#													  						  #	
########################################################
file(GLOB VioHdrs				"./*.h" "./*.hpp")
file(GLOB VioFactorHdrs			"./factor/*.h" "./factor/*.hpp")
file(GLOB VioUtilityHdrs		"./utility/*.h" "./utility/*.hpp")
file(GLOB VioInitialHdrs		"./initial/*.h" "./initial/*.hpp")
file(GLOB VioFeatureHdrs		"./feature/*.h" "./feature/*.hpp")
file(GLOB VioLoopClosureHdrs	"./loop_closure/*.h" "./loop_closure/*.hpp")
file(GLOB VioVisualizationHdrs	"./visualization/*.h" "./visualization/*.hpp")

file(GLOB VioSrcs				"./*.cpp")
file(GLOB VioFactorSrcs			"./factor/*.cpp")
file(GLOB VioUtilitySrcs		"./utility/*.cpp")
file(GLOB VioInitialSrcs		"./initial/*.cpp")
file(GLOB VioFeatureSrcs		"./feature/*.cpp")
file(GLOB VioLoopClosureSrcs	"./loop_closure/*.cpp")
file(GLOB VioVisualizationSrcs	"./visualization/*.cpp")

source_group("example"			FILES ${VioSrcs} ${VioHdrs})
source_group("factor"			FILES ${VioFactorSrcs} ${VioFactorHdrs})
source_group("utility" 			FILES ${VioUtilitySrcs} ${VioUtilityHdrs})
source_group("initial" 			FILES ${VioInitialSrcs} ${VioInitialHdrs})
source_group("feature" 			FILES ${VioFeatureSrcs} ${VioFeatureHdrs})
source_group("loop_closure" 	FILES ${VioLoopClosureSrcs} ${VioLoopClosureHdrs})
source_group("visualization"	FILES ${VioVisualizationSrcs} ${VioVisualizationHdrs})

add_library(vins_estimator 
	${VioHdrs} 
	${VioSrcs}
	${VioFactorSrcs}
	${VioFactorHdrs}
	${VioUtilitySrcs}
	${VioUtilityHdrs}
	${VioInitialSrcs}
	${VioInitialHdrs}
	${VioFeatureSrcs}
	${VioFeatureHdrs}
	${VioLoopClosureSrcs}
	${VioLoopClosureHdrs}
	${VioVisualizationSrcs}
	${VioVisualizationHdrs}
)

target_link_libraries(vins_estimator 
	Ceres::ceres
	${DBoW}
    ${OpenCV_LIBRARY}
    Boost::container
    RabbitMQ
    Json
)
