
#import "LoginController.h"
#import <Foundation/Foundation.h>

@implementation LoginController

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    _userNameField.clearButtonMode = UITextFieldViewModeWhileEditing;
    _userNameField.text = @"xinxin";
    _userNameField.textAlignment = NSTextAlignmentCenter;
    
    _passwordField.clearButtonMode = UITextFieldViewModeWhileEditing;
    _passwordField.text = @"xhl6457398yy";
    _passwordField.textAlignment = NSTextAlignmentCenter;
    _passwordField.secureTextEntry = YES;
    
    _hostnameField.clearButtonMode = UITextFieldViewModeWhileEditing;
    _hostnameField.text = @"127.0.0.1";
    _hostnameField.textAlignment = NSTextAlignmentCenter;
}

- (IBAction)touchLoginButton:(id)sender
{
    NSString *username = [NSString stringWithFormat:@"%@", _userNameField.text];
    NSString *password = [NSString stringWithFormat:@"%@", _passwordField.text];
    NSString *hostname = [NSString stringWithFormat:@"%@", _hostnameField.text];
    
    NSArray *keys = [NSArray arrayWithObjects:@"username",@"password",@"hostname",nil];
    NSArray *values = [NSArray arrayWithObjects:username,password,hostname,nil];
    NSDictionary *notificationDict = [NSDictionary dictionaryWithObjects:values forKeys:keys];
        
    [[NSNotificationCenter defaultCenter] postNotificationName:@"loginInfo"
                                                        object:nil
                                                      userInfo:notificationDict];
    
    [self performSegueWithIdentifier:@"ShowMainUI" sender:sender];
}

- (IBAction)touchCancellButton:(id)sender
{
    exit(0);
}

- (IBAction)userNameFieldDoneEdit:(id)sender
{
    [sender resignFirstResponder];
}

- (IBAction)passWordFieldDoneEdit:(id)sender
{
    [sender resignFirstResponder];
}

- (IBAction)hostNameFieldDoneEdit:(id)sender
{
    [sender resignFirstResponder];
}

@end
