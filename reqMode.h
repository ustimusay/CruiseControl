#ifndef REQMODE_H
#define REQMODE_H

typedef enum {
    None          = 0,
    EnableReq     = 1,
    DisableReq    = 2,
    ActivateReq   = 3,
    DeactivateReq = 4,
    ResumeReq     = 5,
    SpeedIncReq   = 6,
    SpeedDecReq   = 7
} reqMode;

#endif /* REQMODE_H */
