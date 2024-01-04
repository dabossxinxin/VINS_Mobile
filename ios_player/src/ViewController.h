
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/videoio/cap_ios.h>

#import <sys/utsname.h>
#import <mach/mach_time.h>

#include "keyframe.h"
#include "loop_closure.h"
#include "keyfame_database.h"

#import "feature_tracker.hpp"
#import "VINS.hpp"
#import "global_param.hpp"
#import "draw_result.hpp"

#import "LoginController.h"
#import <UIKit/UIKit.h>
#import <CoreMotion/CoreMotion.h>

@interface ViewController : UIViewController<CvVideoCameraDelegate,UITextViewDelegate>
{
    BOOL isCapturing;                           // 初始化OK后，该状态置为true后开始处理图像信息
    CvVideoCamera* videoCamera;                 // OpenCV中提供的采集相机图像数据的接口
    cv::Ptr<FeatureTracker> feature_tracker;    // VINS中负责提取图像特征以及跟踪图像特征的接口
    cv::Size frameSize;                         // IPHONE相机采集得到图像的宽度以及高度
    uint64_t prevTime;                          //
    NSCondition *condition;                     // 线程控制条件变量
    
    NSThread *sendImgThread;                    // RabbitMQ数据发送线程
    NSThread *sendImuThread;                    // RabbitMQ数据发送线程
    NSThread *mainThread;                       // VINS前端里程计处理线程
    NSThread *drawThread;                       // IPHONE端可视化线程
    NSThread *didCameraThread;                  // IPHONE端相机采集数据线程
    NSThread *loopDetectThread;                 // VINS后端闭环检测线程
    NSThread *loopRefineThread;                 // VINS后端闭环矫正线程
    NSThread *connectAMQPThread;                // AMQP登陆线程，登陆成功后线程关闭
}

@property (nonatomic, strong) IBOutlet UIImageView *resultImageView;
@property (nonatomic, strong) IBOutlet UIImageView *rawImageView;

@property (weak, nonatomic) IBOutlet UIButton *loopButton;
@property (weak, nonatomic) IBOutlet UIButton *reinitButton;

@property (weak, nonatomic) IBOutlet UISegmentedControl *switchUI;

@property (nonatomic, strong) CvVideoCamera *videoCamera;
@property (nonatomic) BOOL switchUIAREnabled;

@property (copy, nonatomic) NSString *username;     // rabbitmq登陆账户
@property (copy, nonatomic) NSString *password;     // rabbitmq登陆密码
@property (copy, nonatomic) NSString *hostname;     // rabbitmq登陆地址

- (void)showFeatureStaticInfo;
- (void)showLogInfo:(NSString*)info;
- (void)showRawImageInfo:(UIImage*)image;
- (void)showResultImageInfo:(UIImage*)image;

// 缓存需要录制的IMU数据结构
struct IMU_MSG
{
    NSTimeInterval header;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};

// 存储图像中提取的特征点
struct IMG_MSG
{
    NSTimeInterval header;
    std::map<int, Eigen::Vector3d> point_clouds;
};

// 缓存需要录制的图像数据结构
struct IMG_DATA
{
    NSTimeInterval header;
    UIImage *image;
};

struct IMG_DATA_CACHE
{
    NSTimeInterval header;
    cv::Mat equalizeImage;
};

struct VINS_DATA_CACHE
{
    NSTimeInterval header;
    Eigen::Vector3f P;
    Eigen::Matrix3f R;
};

typedef std::shared_ptr<IMU_MSG const>  ImuConstPtr;
typedef std::shared_ptr<IMG_MSG const>  ImgConstPtr;
@end
