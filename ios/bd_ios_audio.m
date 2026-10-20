/* Bush Drift on iOS (session 06): the audio session. raylib's audio library asks for "play and record", which
 * ignores the silent switch; a game wants Ambient: the switch mutes it and other apps' music keeps playing. */
#import <AVFoundation/AVFoundation.h>
#include "bd_ios.h"

void bd_ios_audio_ambient(void)
{
    @autoreleasepool {
        AVAudioSession *s = [AVAudioSession sharedInstance];
        NSError *err = nil;
        if (![s setCategory:AVAudioSessionCategoryAmbient error:&err]) NSLog(@"ios: audio category: %@", err);
        if (![s setActive:YES error:&err]) NSLog(@"ios: audio session: %@", err);
    }
}
