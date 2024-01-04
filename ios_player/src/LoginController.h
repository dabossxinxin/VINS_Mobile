
#ifndef LoginController_h
#define LoginController_h

#import "ViewController.h"
#import <UIKit/UIKit.h>
#import <sys/utsname.h>

@interface LoginController : UIViewController<UITextViewDelegate>
{
}

@property (weak, nonatomic) IBOutlet UITextField    *userNameField;
@property (weak, nonatomic) IBOutlet UITextField    *passwordField;
@property (weak, nonatomic) IBOutlet UITextField    *hostnameField;
@property (weak, nonatomic) IBOutlet UIButton       *cancelButton;
@property (weak, nonatomic) IBOutlet UIButton       *loginButton;

@end

#endif /* LoginController_h */
